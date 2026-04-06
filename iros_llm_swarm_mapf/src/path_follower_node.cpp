#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_path.hpp"

// ---------------------------------------------------------------------------

class PathFollowerNode : public rclcpp::Node
{
  using FollowPath  = nav2_msgs::action::FollowPath;
  using GoalHandle  = rclcpp_action::ClientGoalHandle<FollowPath>;

 public:
  PathFollowerNode() : Node("path_follower")
  {
    declare_parameter("robot_id",               0);
    declare_parameter("path_frame",             std::string("map"));
    // Если робот приехал к waypoint раньше расписания больше чем на это
    // значение — притормаживает (отправляет только часть пути до этой точки
    // и ждёт). Уменьши до 0.0 чтобы никогда не ждать.
    declare_parameter("schedule_tolerance_sec", 0.5);

    const int robot_id = get_parameter("robot_id").as_int();
    ns_    = "robot_" + std::to_string(robot_id);
    frame_ = get_parameter("path_frame").as_string();
    tol_   = get_parameter("schedule_tolerance_sec").as_double();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/" + ns_ + "/mapf_path", 10,
        [this](const nav_msgs::msg::Path::SharedPtr msg) { on_path(msg); });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/" + ns_ + "/odom", 10,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          current_x_ = msg->pose.pose.position.x;
          current_y_ = msg->pose.pose.position.y;
          have_pose_ = true;
        });

    ac_ = rclcpp_action::create_client<FollowPath>(
        this, "/" + ns_ + "/follow_path");

    RCLCPP_INFO(get_logger(), "[%s] path_follower ready", ns_.c_str());
  }

 private:
  // ------------------------------------------------------------------
  void on_path(const nav_msgs::msg::Path::SharedPtr msg)
  {
    // Empty path = cancel signal (replanner stops deviated robots)
    if (msg->poses.empty()) {
      RCLCPP_INFO(get_logger(), "[%s] received empty path — cancelling", ns_.c_str());
      cancel_current();
      pending_path_.reset();
      return;
    }

    RCLCPP_INFO(get_logger(), "[%s] new path: %zu waypoints",
                ns_.c_str(), msg->poses.size());

    // Отменяем текущее движение если есть
    cancel_current();

    pending_path_ = msg;
    wp_idx_       = 0;

    if (!have_pose_) {
      // Ждём первой одометрии
      start_timer_ = create_wall_timer(
          std::chrono::milliseconds(100),
          [this]() {
            if (!have_pose_) return;
            start_timer_->cancel();
            start_timer_.reset();
            send_next_chunk();
          });
    } else {
      send_next_chunk();
    }
  }

  // ------------------------------------------------------------------
  // Отправляем путь от текущего wp_idx_ до ближайшей "точки ожидания".
  //
  // Логика: идём по waypoints вперёд пока не найдём точку где по расписанию
  // нужно подождать (scheduled_time существенно в будущем). Отправляем путь
  // до этой точки включительно. Когда DWB доедет — ждём до scheduled_time
  // и отправляем следующий chunk.
  //
  // Если ни одной точки ожидания нет — отправляем весь оставшийся путь
  // сразу. Это самый частый случай (роботы не опережают расписание).
  // ------------------------------------------------------------------
  void send_next_chunk()
  {
    if (!pending_path_ || wp_idx_ >= pending_path_->poses.size()) {
      RCLCPP_INFO(get_logger(), "[%s] path complete", ns_.c_str());
      pending_path_.reset();
      return;
    }

    if (!ac_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_ERROR(get_logger(),
                   "[%s] follow_path server not available", ns_.c_str());
      return;
    }

    const rclcpp::Time now_t = now();
    const auto& poses = pending_path_->poses;

    // Ищем первую точку где надо ждать
    size_t stop_idx = poses.size() - 1;  // по умолчанию — до конца
    rclcpp::Time stop_sched{0, 0, RCL_ROS_TIME};

    for (size_t i = wp_idx_; i < poses.size(); ++i) {
      const rclcpp::Time sched(poses[i].header.stamp);
      const double wait = (sched - now_t).seconds();
      if (wait > tol_) {
        stop_idx   = i;
        stop_sched = sched;
        break;
      }
    }

    // Строим подпуть wp_idx_..stop_idx
    nav_msgs::msg::Path chunk;
    chunk.header.frame_id = frame_;
    chunk.header.stamp    = now_t;

    // Первая точка — текущая позиция (чтобы DWB не дёргался)
    geometry_msgs::msg::PoseStamped cur_ps;
    cur_ps.header.frame_id = frame_;
    cur_ps.header.stamp    = now_t;
    cur_ps.pose.position.x = current_x_;
    cur_ps.pose.position.y = current_y_;
    cur_ps.pose.orientation.w = 1.0;
    chunk.poses.push_back(cur_ps);

    for (size_t i = wp_idx_; i <= stop_idx; ++i) {
      chunk.poses.push_back(poses[i]);
    }

    // Выставляем ориентацию вдоль пути
    fix_orientations(chunk);

    RCLCPP_DEBUG(get_logger(),
                "[%s] sending chunk wp %zu..%zu/%zu  -> (%.2f, %.2f)%s",
                ns_.c_str(), wp_idx_, stop_idx, poses.size() - 1,
                poses[stop_idx].pose.position.x,
                poses[stop_idx].pose.position.y,
                stop_idx < poses.size() - 1 ? "  [hold ahead]" : "");

    FollowPath::Goal goal;
    goal.path = chunk;

    auto opts = rclcpp_action::Client<FollowPath>::SendGoalOptions{};

    opts.goal_response_callback =
        [this](const GoalHandle::SharedPtr& gh) {
          if (!gh) {
            RCLCPP_WARN(get_logger(), "[%s] goal rejected", ns_.c_str());
          } else {
            current_gh_ = gh;
          }
        };

    opts.result_callback =
        [this, stop_idx, stop_sched](const GoalHandle::WrappedResult&) {
          current_gh_.reset();
          on_chunk_done(stop_idx, stop_sched);
        };

    ac_->async_send_goal(goal, opts);
    wp_idx_ = stop_idx + 1;
  }

  void on_chunk_done(size_t stop_idx, const rclcpp::Time& stop_sched)
  {
    // Если путь закончился
    if (!pending_path_ || stop_idx >= pending_path_->poses.size() - 1) {
      RCLCPP_INFO(get_logger(), "[%s] path complete", ns_.c_str());
      pending_path_.reset();
      return;
    }

    // Если нужно ждать по расписанию
    const double wait = (stop_sched - now()).seconds();
    if (wait > 0.02) {
      RCLCPP_DEBUG(get_logger(),
                  "[%s] holding %.2fs at wp %zu per PBS schedule",
                  ns_.c_str(), wait, stop_idx);
      hold_timer_ = create_wall_timer(
          std::chrono::duration<double>(wait),
          [this]() {
            hold_timer_->cancel();
            hold_timer_.reset();
            send_next_chunk();
          });
    } else {
      send_next_chunk();
    }
  }

  // ------------------------------------------------------------------
  void cancel_current()
  {
    if (hold_timer_) { hold_timer_->cancel(); hold_timer_.reset(); }
    if (start_timer_) { start_timer_->cancel(); start_timer_.reset(); }
    if (current_gh_) {
      ac_->async_cancel_goal(current_gh_);
      current_gh_.reset();
    }
  }

  // Выставляем ориентацию каждой точки пути вдоль вектора к следующей
  static void fix_orientations(nav_msgs::msg::Path& path)
  {
    for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
      const double dx = path.poses[i+1].pose.position.x
                      - path.poses[i].pose.position.x;
      const double dy = path.poses[i+1].pose.position.y
                      - path.poses[i].pose.position.y;
      if (std::abs(dx) > 1e-4 || std::abs(dy) > 1e-4) {
        const double yaw = std::atan2(dy, dx);
        path.poses[i].pose.orientation.z = std::sin(yaw * 0.5);
        path.poses[i].pose.orientation.w = std::cos(yaw * 0.5);
      }
    }
    // Последняя точка — ориентация как у предпоследней
    if (path.poses.size() >= 2) {
      path.poses.back().pose.orientation =
          path.poses[path.poses.size() - 2].pose.orientation;
    }
  }

  // ------------------------------------------------------------------
  std::string  ns_, frame_;
  double       tol_       = 0.5;
  bool         have_pose_ = false;
  double       current_x_ = 0.0, current_y_ = 0.0;
  size_t       wp_idx_    = 0;

  std::shared_ptr<const nav_msgs::msg::Path> pending_path_;
  GoalHandle::SharedPtr                      current_gh_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr     path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp_action::Client<FollowPath>::SharedPtr             ac_;

  rclcpp::TimerBase::SharedPtr hold_timer_;
  rclcpp::TimerBase::SharedPtr start_timer_;
};

// ---------------------------------------------------------------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}