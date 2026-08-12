//
// Gravity compensation controller for Panthera HT (single / dual arm).
//
// Architecture (matches the ht_ros2_control hardware interface contract):
//   - For each joint in `joints` (arm joints): claim position/effort
//     command interfaces (+ optional kp/kd) and the position state interface.
//     Every update cycle:
//       * effort = static gravity torque from Pinocchio RNEA (clamped)
//       * position = current measured position (hold in place, prevents the
//         hardware interface from treating an unclaimed/zero position command
//         as a move-to-zero target)
//       * kp/kd = soft gains (weak stiffness hold + gravity feedforward)
//   - For each joint in `hold_joints` (e.g. gripper): only position state +
//     position command, following the current value (keep in place).
//
// The URDF is read from the `urdf_param` parameter (default `robot_description`,
// injected by controller_manager or read from /controller_manager).
//
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "ht_gravity_compensation/gravity_compensation.hpp"

namespace ht_gravity_compensation
{

class GravityCompensationController : public controller_interface::ControllerInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(GravityCompensationController)

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool loadRobotDescription(std::string & urdf_out);
  static double gainAt(const std::vector<double> & gains, size_t index);

  // ---- 参数 ----
  std::vector<std::string> joint_names_;      // 重力补偿关节（臂关节）
  std::vector<std::string> hold_joint_names_; // 仅位置保持关节（夹爪等）
  bool use_pd_{false};                        // 是否写 kp/kd 命令接口
  std::vector<double> hold_kp_;
  std::vector<double> hold_kd_;
  std::vector<double> max_effort_;            // 力矩限幅（Nm），空 = 不限幅
  std::vector<double> gravity_vector_{0.0, 0.0, -9.81};
  std::string urdf_param_name_{"robot_description"};

  // ---- 动力学 ----
  std::unique_ptr<GravityCompensation> gravity_;
  // joint_names_[i] 在模型中的 JointIndex（SIZE_MAX = 模型缺失，力矩置 0）
  std::vector<size_t> model_joint_indices_;

  // ---- 借用的硬件接口（on_activate 时填充） ----
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    joint_position_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
    hold_position_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_position_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_effort_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_kp_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    joint_kd_command_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
    hold_position_command_interface_;
};

}  // namespace ht_gravity_compensation
