// Copyright 2026 — iros_llm_swarm contributors. Apache-2.0.

#include "iros_llm_rviz_panel/action_summary.hpp"

#include <cctype>
#include <charconv>

namespace iros_llm_rviz_panel
{

namespace
{

std::optional<int64_t> parseInt(std::string_view s)
{
  if (s.empty()) {
    return std::nullopt;
  }
  int64_t out = 0;
  const char * begin = s.data();
  const char * end   = s.data() + s.size();
  auto [ptr, ec] = std::from_chars(begin, end, out);
  if (ec != std::errc{} || ptr != end) {
    return std::nullopt;
  }
  return out;
}

bool isSpace(char c)
{
  return std::isspace(static_cast<unsigned char>(c)) != 0;
}

}  // namespace

ActionSummary parseActionSummary(std::string_view line)
{
  ActionSummary s;
  if (line.empty()) {
    return s;
  }

  const auto lb = line.find('[');
  if (lb == std::string_view::npos) {
    return s;
  }
  const auto rb = line.find(']', lb + 1);
  if (rb == std::string_view::npos) {
    return s;
  }

  // Tokenize the bracketed payload by whitespace; each token is `key=value`.
  const auto inside = line.substr(lb + 1, rb - lb - 1);
  size_t pos = 0;
  while (pos < inside.size()) {
    while (pos < inside.size() && isSpace(inside[pos])) {
      ++pos;
    }
    if (pos >= inside.size()) {
      break;
    }
    const size_t start = pos;
    while (pos < inside.size() && !isSpace(inside[pos])) {
      ++pos;
    }
    const auto token = inside.substr(start, pos - start);
    const auto eq = token.find('=');
    if (eq == std::string_view::npos || eq == 0) {
      continue;
    }
    const auto key = token.substr(0, eq);
    auto val       = token.substr(eq + 1);

    if (key == "t") {
      // strip "ms" suffix if present
      if (val.size() >= 2 && val.substr(val.size() - 2) == "ms") {
        val.remove_suffix(2);
      }
      s.elapsed_ms = parseInt(val);
    } else if (key == "status") {
      if (!val.empty()) {
        s.status = std::string(val);
      }
    } else if (key == "arrived") {
      if (auto v = parseInt(val)) {
        s.arrived = static_cast<int>(*v);
      }
    } else if (key == "active") {
      if (auto v = parseInt(val)) {
        s.active = static_cast<int>(*v);
      }
    } else if (key == "stall") {
      if (auto v = parseInt(val)) {
        s.stall = static_cast<int>(*v);
      }
    } else if (key == "replans") {
      if (auto v = parseInt(val)) {
        s.replans = static_cast<int>(*v);
      }
    }
    // Unknown keys are silently ignored — forward-compat with future fields.
  }

  // Tail — anything after the closing bracket, leading whitespace stripped.
  if (rb + 1 < line.size()) {
    auto tail = line.substr(rb + 1);
    const auto first = tail.find_first_not_of(" \t");
    if (first != std::string_view::npos) {
      s.event_tail = std::string(tail.substr(first));
    }
  }

  return s;
}

}  // namespace iros_llm_rviz_panel
