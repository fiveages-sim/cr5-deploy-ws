#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <string_view>

namespace wujihand2_ros2_control
{
namespace hand2
{

inline constexpr std::size_t kJointCount = 20;

/** Assumed SDK index 0..19 ↔ hand2.yaml joint order (calibrate on device). */
inline constexpr std::array<std::string_view, kJointCount> kJointNameSuffixes = {
  "thumb_cmc_flex",
  "thumb_cmc_abd",
  "thumb_mcp",
  "thumb_ip",
  "index_finger_mcp_flex",
  "index_finger_mcp_abd",
  "index_finger_pip",
  "index_finger_dip",
  "middle_finger_mcp_flex",
  "middle_finger_mcp_abd",
  "middle_finger_pip",
  "middle_finger_dip",
  "ring_mcp_flex",
  "ring_mcp_abd",
  "ring_pip",
  "ring_dip",
  "pinky_mcp_flex",
  "pinky_mcp_abd",
  "pinky_pip",
  "pinky_dip",
};

inline constexpr double kDefaultMitKp = 3.0;
inline constexpr double kDefaultMitKd = 0.05;
inline constexpr double kDefaultEffortLimitA = 1.5;

/** Conservative default position limits (rad) when URDF has none. */
inline constexpr std::array<double, kJointCount> kDefaultLowerLimits = {
  -1.187, -1.484, -1.047, -1.047,
  -1.047, -0.698, -1.047, -1.047,
  -1.047, -0.698, -1.047, -1.047,
  -1.047, -0.698, -1.047, -1.047,
  -1.047, -0.698, -1.047, -1.047,
};

inline constexpr std::array<double, kJointCount> kDefaultUpperLimits = {
  1.291, 0.698, 1.570, 1.570,
  1.570, 0.698, 2.094, 1.570,
  1.570, 0.698, 2.094, 1.570,
  1.570, 0.698, 2.094, 1.570,
  1.570, 0.698, 2.094, 1.570,
};

inline bool has_suffix(const std::string & value, std::string_view suffix)
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

/** Map ROS joint name → assumed SDK index; returns kJointCount if unknown. */
inline std::size_t sdk_index_for_joint_name(const std::string & joint_name)
{
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (has_suffix(joint_name, kJointNameSuffixes[i])) {
      return i;
    }
  }
  return kJointCount;
}

}  // namespace hand2
}  // namespace wujihand2_ros2_control
