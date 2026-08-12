//
// GravityCompensationController implementation.
//
#include "ht_gravity_compensation/gravity_compensation_controller.hpp"

#include <algorithm>
#include <limits>
#include <tuple>
#include <unordered_map>

#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ht_gravity_compensation::GravityCompensationController,
  controller_interface::ControllerInterface);

namespace ht_gravity_compensation
{

controller_interface::CallbackReturn GravityCompensationController::on_init()
{
  try
  {
    joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
    hold_joint_names_ = auto_declare<std::vector<std::string>>("hold_joints", hold_joint_names_);
    use_pd_ = auto_declare<bool>("use_pd", use_pd_);
    hold_kp_ = auto_declare<std::vector<double>>("hold_kp", hold_kp_);
    hold_kd_ = auto_declare<std::vector<double>>("hold_kd", hold_kd_);
    max_effort_ = auto_declare<std::vector<double>>("max_effort", max_effort_);
    gravity_vector_ = auto_declare<std::vector<double>>("gravity_vector", gravity_vector_);
    urdf_param_name_ = auto_declare<std::string>("urdf_param", urdf_param_name_);
    // robot_description 通过 --params-file（launch 写入的控制器参数文件）传入：
    // rclcpp 的 override 只在参数被声明时落地，未声明的参数会被忽略，
    // 因此必须在此声明（值取自 override，缺省为空字符串）。
    auto_declare<std::string>(urdf_param_name_, "");
  }
  catch (const std::exception & e)
  {
    RCLCPP_FATAL(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool GravityCompensationController::loadRobotDescription(std::string & urdf_out)
{
  // 1) 自身参数（controller_manager 注入的 robot_description，launch 已直接写入控制器参数）
  if (get_node()->has_parameter(urdf_param_name_))
  {
    rclcpp::Parameter param;
    if (get_node()->get_parameter(urdf_param_name_, param) &&
      param.get_type() == rclcpp::ParameterType::PARAMETER_STRING &&
      !param.as_string().empty())
    {
      urdf_out = param.as_string();
      return true;
    }
  }
  // 2) 回退：从 controller_manager 节点读取。
  //    必须使用独立临时节点：SyncParametersClient 会创建自己的 SingleThreadedExecutor
  //    并把节点加入其中，而控制器节点已属于 controller_manager 的 executor（会报错）。
  //    加 wait_for_service 超时：参数服务不可达（如 ns 下服务名不匹配）时快速返回，
  //    避免 configure 永久阻塞（executor 死锁）。
  try
  {
    auto probe_node = std::make_shared<rclcpp::Node>("ht_gravity_compensation_urdf_probe");
    auto client = std::make_shared<rclcpp::SyncParametersClient>(
      probe_node, "/controller_manager");
    if (!client->wait_for_service(std::chrono::milliseconds(500)))
    {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Parameter services of /controller_manager not available within 500ms; "
        "cannot fall back to reading '%s' from it", urdf_param_name_.c_str());
      return false;
    }
    if (client->has_parameter(urdf_param_name_))
    {
      urdf_out = client->get_parameter<std::string>(urdf_param_name_);
      return !urdf_out.empty();
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_WARN(
      get_node()->get_logger(), "Failed to read '%s' from /controller_manager: %s",
      urdf_param_name_.c_str(), e.what());
  }
  return false;
}

controller_interface::CallbackReturn GravityCompensationController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (joint_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'joints' must not be empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (gravity_vector_.size() != 3)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Parameter 'gravity_vector' must have exactly 3 elements");
    return controller_interface::CallbackReturn::ERROR;
  }

  std::string urdf;
  if (!loadRobotDescription(urdf))
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Failed to obtain robot URDF from parameter '%s' (make sure ros2_control_node "
      "receives 'robot_description', e.g. via the launch file)",
      urdf_param_name_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  try
  {
    gravity_ = std::make_unique<GravityCompensation>(urdf, gravity_vector_);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Failed to build Pinocchio model from URDF: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (!gravity_->isValid())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Pinocchio model is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  // 建立 joint_names_ → 模型 JointIndex 映射
  const auto & model = gravity_->getModel();
  model_joint_indices_.clear();
  model_joint_indices_.reserve(joint_names_.size());
  for (const auto & name : joint_names_)
  {
    if (model.existJointName(name))
    {
      model_joint_indices_.push_back(model.getJointId(name));
    }
    else
    {
      model_joint_indices_.push_back(std::numeric_limits<size_t>::max());
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Joint '%s' not found in the URDF model; its gravity torque will be 0.0 "
        "(only affects compensation, position hold still applies)", name.c_str());
    }
  }

  // 提示模型中缺少状态来源的关节（如 mimic 的 gripper_joint2）：q 置 0
  for (pinocchio::JointIndex jid = 1;
    jid < static_cast<pinocchio::JointIndex>(model.names.size()); ++jid)
  {
    const auto & jm = model.joints[jid];
    if (jm.nq() != 1)
    {
      continue;
    }
    const auto & name = model.names[jid];
    const bool has_state =
      std::find(joint_names_.begin(), joint_names_.end(), name) != joint_names_.end() ||
      std::find(hold_joint_names_.begin(), hold_joint_names_.end(), name) !=
        hold_joint_names_.end();
    if (!has_state)
    {
      RCLCPP_INFO(
        get_node()->get_logger(),
        "Model joint '%s' has no state source (e.g. mimic joint); its q is set to 0 "
        "in RNEA (negligible effect)", name.c_str());
    }
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Configured: %zu compensated joints, %zu hold joints, use_pd=%s, "
    "Pinocchio model nq=%zu (gravity [%.2f, %.2f, %.2f])",
    joint_names_.size(), hold_joint_names_.size(), use_pd_ ? "true" : "false",
    gravity_->getNumJoints(), gravity_vector_[0], gravity_vector_[1], gravity_vector_[2]);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
GravityCompensationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : joint_names_)
  {
    config.names.push_back(name + "/position");
    config.names.push_back(name + "/effort");
    if (use_pd_)
    {
      config.names.push_back(name + "/kp");
      config.names.push_back(name + "/kd");
    }
  }
  for (const auto & name : hold_joint_names_)
  {
    config.names.push_back(name + "/position");
  }
  return config;
}

controller_interface::InterfaceConfiguration
GravityCompensationController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & name : joint_names_)
  {
    config.names.push_back(name + "/position");
  }
  for (const auto & name : hold_joint_names_)
  {
    config.names.push_back(name + "/position");
  }
  return config;
}

controller_interface::CallbackReturn GravityCompensationController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto claim_command = [this](
    const std::string & full_name,
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> & out)
  {
    for (auto & interface : command_interfaces_)
    {
      // Jazzy: get_name() 返回完整名（如 "left_joint1/position"），
      // get_interface_name() 只返回接口部分（"position"），无法区分关节。
      if (interface.get_name() == full_name)
      {
        out.emplace_back(interface);
        return true;
      }
    }
    return false;
  };

  auto claim_state = [this](
    const std::string & full_name,
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> & out)
  {
    for (auto & interface : state_interfaces_)
    {
      if (interface.get_name() == full_name)
      {
        out.emplace_back(interface);
        return true;
      }
    }
    return false;
  };

  // 清空上次借用的接口
  joint_position_state_interface_.clear();
  hold_position_state_interface_.clear();
  joint_position_command_interface_.clear();
  joint_effort_command_interface_.clear();
  joint_kp_command_interface_.clear();
  joint_kd_command_interface_.clear();
  hold_position_command_interface_.clear();

  // 臂关节：position 状态 + position/effort 命令（必需）；kp/kd（可选）
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    const std::string & base = joint_names_[i];
    const bool state_ok = claim_state(base + "/position", joint_position_state_interface_);
    const bool pos_ok = claim_command(base + "/position", joint_position_command_interface_);
    const bool eff_ok = claim_command(base + "/effort", joint_effort_command_interface_);
    if (!state_ok || !pos_ok || !eff_ok)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to claim interfaces for joint '%s' (state=%s position=%s effort=%s)",
        base.c_str(), state_ok ? "ok" : "missing", pos_ok ? "ok" : "missing",
        eff_ok ? "ok" : "missing");
      return controller_interface::CallbackReturn::ERROR;
    }
    if (use_pd_)
    {
      const bool kp_ok = claim_command(base + "/kp", joint_kp_command_interface_);
      const bool kd_ok = claim_command(base + "/kd", joint_kd_command_interface_);
      if (!kp_ok || !kd_ok)
      {
        RCLCPP_WARN(
          get_node()->get_logger(),
          "kp/kd command interfaces not available for joint '%s'; falling back to "
          "hardware default gains", base.c_str());
        joint_kp_command_interface_.clear();
        joint_kd_command_interface_.clear();
        use_pd_ = false;
      }
    }
  }

  // 保持关节（夹爪）：position 状态 + position 命令
  for (size_t i = 0; i < hold_joint_names_.size(); ++i)
  {
    const std::string & base = hold_joint_names_[i];
    const bool state_ok = claim_state(base + "/position", hold_position_state_interface_);
    const bool pos_ok = claim_command(base + "/position", hold_position_command_interface_);
    if (!state_ok || !pos_ok)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Failed to claim position interfaces for hold joint '%s' (state=%s position=%s)",
        base.c_str(), state_ok ? "ok" : "missing", pos_ok ? "ok" : "missing");
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "Activated: %zu arm joints compensated, %zu hold joints kept in place",
    joint_position_command_interface_.size(), hold_position_command_interface_.size());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_position_state_interface_.clear();
  hold_position_state_interface_.clear();
  joint_position_command_interface_.clear();
  joint_effort_command_interface_.clear();
  joint_kp_command_interface_.clear();
  joint_kd_command_interface_.clear();
  hold_position_command_interface_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

double GravityCompensationController::gainAt(const std::vector<double> & gains, size_t index)
{
  return gains.empty() ? 0.0 : gains[std::min(index, gains.size() - 1)];
}

controller_interface::return_type GravityCompensationController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 1) 读取当前关节位置（名称 → 值）
  std::unordered_map<std::string, double> state_values;
  state_values.reserve(joint_names_.size() + hold_joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    state_values[joint_names_[i]] =
      joint_position_state_interface_[i].get().get_optional().value_or(0.0);
  }
  for (size_t i = 0; i < hold_joint_names_.size(); ++i)
  {
    state_values[hold_joint_names_[i]] =
      hold_position_state_interface_[i].get().get_optional().value_or(0.0);
  }

  // 2) 组装模型 q：按名称映射（1-DoF 关节）；无状态来源的关节（如 mimic）保持 0
  const auto & model = gravity_->getModel();
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);
  for (pinocchio::JointIndex jid = 1;
    jid < static_cast<pinocchio::JointIndex>(model.names.size()); ++jid)
  {
    const auto & joint_model = model.joints[jid];
    if (joint_model.nq() != 1)
    {
      continue;
    }
    const auto it = state_values.find(model.names[jid]);
    if (it != state_values.end())
    {
      q[joint_model.idx_q()] = it->second;
    }
  }

  // 3) 静态重力矩（零速零加速 RNEA）
  const Eigen::VectorXd tau = gravity_->calculateStaticTorques(q);

  // 4) 臂关节：effort = 重力矩（限幅）；position = 当前值（保位，防硬件回零）；
  //    kp/kd = 软刚度（可选）
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    double effort = 0.0;
    const size_t model_index = model_joint_indices_[i];
    if (model_index != std::numeric_limits<size_t>::max())
    {
      const auto & joint_model = model.joints[model_index];
      effort = tau[joint_model.idx_v()];
    }
    if (!max_effort_.empty())
    {
      const double limit = gainAt(max_effort_, i);
      effort = std::clamp(effort, -limit, limit);
    }

    std::ignore = joint_effort_command_interface_[i].get().set_value(effort);
    std::ignore = joint_position_command_interface_[i].get().set_value(
      state_values[joint_names_[i]]);

    if (use_pd_)
    {
      if (i < joint_kp_command_interface_.size())
      {
        std::ignore = joint_kp_command_interface_[i].get().set_value(gainAt(hold_kp_, i));
      }
      if (i < joint_kd_command_interface_.size())
      {
        std::ignore = joint_kd_command_interface_[i].get().set_value(gainAt(hold_kd_, i));
      }
    }
  }

  // 5) 保持关节（夹爪）：位置跟随当前值
  for (size_t i = 0; i < hold_joint_names_.size(); ++i)
  {
    std::ignore = hold_position_command_interface_[i].get().set_value(
      hold_position_state_interface_[i].get().get_optional().value_or(0.0));
  }

  return controller_interface::return_type::OK;
}

}  // namespace ht_gravity_compensation
