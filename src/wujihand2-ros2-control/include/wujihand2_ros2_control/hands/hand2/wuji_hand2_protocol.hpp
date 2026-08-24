#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
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

/** Map ROS joint name → flat SDK index 0..19; returns kJointCount if unknown. */
inline std::size_t sdk_index_for_joint_name(const std::string & joint_name)
{
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (has_suffix(joint_name, kJointNameSuffixes[i])) {
      return i;
    }
  }
  return kJointCount;
}

/**
 * joint_states `nid` on the wire: bus * 5 + node (1-based), range 1..24 with
 * tactile slots at 5/10/15/20. Maps to flat finger-major index 0..19 used by
 * joint_command and joint_label. Returns kJointCount if invalid.
 */
inline constexpr std::array<std::size_t, 26> kJointStatesNidToFlat = {
  kJointCount,                     // 0 — unused
  0, 1, 2, 3, kJointCount,         // 1..5   thumb + tactile@5
  4, 5, 6, 7, kJointCount,         // 6..10  index + tactile@10
  8, 9, 10, 11, kJointCount,       // 11..15 middle + tactile@15
  12, 13, 14, 15, kJointCount,     // 16..20 ring + tactile@20
  16, 17, 18, 19, kJointCount,     // 21..25 pinky + tactile@25
};

inline std::size_t flat_index_from_joint_states_nid(uint8_t nid)
{
  if (nid >= kJointStatesNidToFlat.size()) {
    return kJointCount;
  }
  return kJointStatesNidToFlat[nid];
}

}  // namespace hand2
}  // namespace wujihand2_ros2_control
