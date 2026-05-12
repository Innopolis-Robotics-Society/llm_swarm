#include "nav2_map_server/map_server.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace iros_llm_swarm_costmap_plugins {

class ZoneMapServer : public nav2_map_server::MapServer
{
public:
  explicit ZoneMapServer(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : MapServer(options) {}

protected:
  nav2_util::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override
  {
    auto result = MapServer::on_configure(state);
    if (result != nav2_util::CallbackReturn::SUCCESS) return result;

    const std::string yaml_file = get_parameter("yaml_filename").as_string();
    const std::string zone_pgm = find_zone_pgm(yaml_file);
    if (!zone_pgm.empty()) {
      overlay_zones(zone_pgm);
    }
    return result;
  }

private:
  std::string find_zone_pgm(const std::string & yaml_path)
  {
    try {
      YAML::Node doc = YAML::LoadFile(yaml_path);
      if (doc["zone_map"]) {
        const std::string rel = doc["zone_map"].as<std::string>();
        return (std::filesystem::path(yaml_path).parent_path() / rel).string();
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Failed to parse zone_map from %s: %s",
        yaml_path.c_str(), e.what());
    }
    return {};
  }

  struct Pgm {
    int width = 0, height = 0, max_val = 255;
    std::vector<uint8_t> pixels;
  };

  static bool read_pgm(const std::string & path, Pgm & out)
  {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    std::string magic;
    f >> magic >> out.width >> out.height >> out.max_val;
    if (f.fail() || (magic != "P5" && magic != "P2")) return false;
    f.get();
    out.pixels.resize(out.width * out.height);
    if (magic == "P5") {
      f.read(reinterpret_cast<char *>(out.pixels.data()), out.pixels.size());
    } else {
      for (auto & p : out.pixels) { int v; f >> v; p = static_cast<uint8_t>(v); }
    }
    return !f.fail();
  }

  void overlay_zones(const std::string & pgm_path)
  {
    Pgm img;
    if (!read_pgm(pgm_path, img)) {
      RCLCPP_ERROR(get_logger(), "Failed to read zone PGM: %s", pgm_path.c_str());
      return;
    }
    const int map_w = static_cast<int>(msg_.info.width);
    const int map_h = static_cast<int>(msg_.info.height);
    if (img.width != map_w || img.height != map_h) {
      RCLCPP_ERROR(get_logger(),
        "Zone PGM %dx%d does not match map %dx%d — skipping",
        img.width, img.height, map_w, map_h);
      return;
    }
    int applied = 0;
    for (int pgm_row = 0; pgm_row < img.height; ++pgm_row) {
      const int map_row = img.height - 1 - pgm_row;  // PGM row 0 = top; OccupancyGrid row 0 = bottom
      for (int col = 0; col < img.width; ++col) {
        const size_t map_idx = static_cast<size_t>(map_row * map_w + col);
        if (msg_.data[map_idx] == 100) continue;  // wall — never overwrite
        const uint8_t px = img.pixels[pgm_row * img.width + col];
        if (px == 255) continue;  // white = no zone
        const int8_t v = static_cast<int8_t>(std::round((1.0 - px / 255.0) * 99.0));
        if (v > 0) { msg_.data[map_idx] = v; ++applied; }
      }
    }
    RCLCPP_INFO(get_logger(), "Zone overlay: %d cells from %s", applied, pgm_path.c_str());
  }
};

}  // namespace iros_llm_swarm_costmap_plugins

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<iros_llm_swarm_costmap_plugins::ZoneMapServer>(options);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
