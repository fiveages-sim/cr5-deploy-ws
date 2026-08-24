#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <wuji_sdk.h>

#include "wujihand2_ros2_control/hands/hand2/wuji_hand2_protocol.hpp"

namespace wujihand2_ros2_control
{

class WujiHand2Hardware : public hardware_interface::SystemInterface
{
public:
  WujiHand2Hardware() = default;
  ~WujiHand2Hardware() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr>
  on_export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr std::size_t kJointCount = hand2::kJointCount;

  void load_parameters();
  bool validate_joint_interfaces() const;
  bool build_joint_maps();

  bool connect_device();
  void disconnect_device();
  bool apply_mit_and_effort_limit();
  bool enable_motors();
  bool disable_motors();
  void safe_shutdown();

  bool start_state_subscription();
  void stop_state_subscription();
  static void on_joint_states_callback(
    WujiFrameKind kind, const WujiJointStateFrame * frame, void * user_data);
  bool wait_initial_feedback(std::chrono::milliseconds timeout);
  void initialize_state_from_feedback();

  bool open_command_publisher();
  void close_command_publisher();
  bool send_joint_commands(const std::array<double, kJointCount> & ros_positions);
  bool command_changed(const std::array<double, kJointCount> & ros_positions) const;

  static bool parse_bool(const std::string & value, bool default_value);
  static int parse_int(const std::string & value, int default_value);
  static double parse_double(const std::string & value, double default_value);
  static double initial_value_for_joint(const hardware_interface::ComponentInfo & joint);

  std::string hand_side_{"left"};
  std::string serial_number_;
  std::string device_address_;
  double mit_kp_{hand2::kDefaultMitKp};
  double mit_kd_{hand2::kDefaultMitKd};
  double effort_limit_{hand2::kDefaultEffortLimitA};
  bool read_feedback_{true};
  bool require_initial_feedback_{true};
  double command_deadband_{0.0};
  int connect_timeout_ms_{5000};
  double enable_timeout_s_{5.0};

  WujiDevice * device_{nullptr};
  WujiSub * state_sub_{nullptr};
  WujiJointCommandPublisher * command_pub_{nullptr};

  std::vector<std::string> joint_names_;
  /** ros_index -> sdk_index (0..19) */
  std::array<std::size_t, kJointCount> ros_to_sdk_{};
  /** sdk_index -> ros_index */
  std::array<std::size_t, kJointCount> sdk_to_ros_{};

  std::array<double, kJointCount> lower_limits_{};
  std::array<double, kJointCount> upper_limits_{};
  std::array<double, kJointCount> hw_positions_{};
  std::array<double, kJointCount> hw_velocities_{};
  std::array<double, kJointCount> hw_efforts_{};
  std::array<double, kJointCount> hw_commands_pos_{};
  std::array<double, kJointCount> last_sent_commands_{};

  std::mutex feedback_mutex_;
  std::condition_variable feedback_cv_;
  std::array<double, kJointCount> feedback_positions_{};
  std::array<double, kJointCount> feedback_velocities_{};
  std::array<double, kJointCount> feedback_efforts_{};
  /** Per-frame online mask: set from joint_states entries (nid); cleared each callback. */
  std::array<bool, kJointCount> feedback_online_{};
  bool feedback_valid_{false};

  std::atomic_bool command_ready_{false};
  bool last_sent_valid_{false};
  bool shutting_down_{false};
};

}  // namespace wujihand2_ros2_control
