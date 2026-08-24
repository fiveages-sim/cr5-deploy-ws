#include "wujihand2_ros2_control/hands/hand2/wuji_hand2_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <thread>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace wujihand2_ros2_control
{
namespace
{
constexpr auto kLoggerName = "WujiHand2Hardware";
constexpr const char * kDeviceAlias = "wuji_hand2";

std::string status_message(WujiStatus status)
{
  const char * err = wuji_last_error();
  if (err != nullptr && err[0] != '\0') {
    return std::string(err) + " (status=" + std::to_string(static_cast<int>(status)) + ")";
  }
  return "status=" + std::to_string(static_cast<int>(status));
}
}  // namespace

WujiHand2Hardware::~WujiHand2Hardware()
{
  safe_shutdown();
}

hardware_interface::CallbackReturn WujiHand2Hardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kJointCount) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Wuji Hand2 hardware expects exactly %zu joints, got %zu", kJointCount,
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  load_parameters();

  joint_names_.clear();
  joint_names_.reserve(kJointCount);
  for (const auto & joint : info_.joints) {
    joint_names_.push_back(joint.name);
  }

  if (!validate_joint_interfaces()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!build_joint_maps()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  lower_limits_ = hand2::kDefaultLowerLimits;
  upper_limits_ = hand2::kDefaultUpperLimits;

  for (std::size_t i = 0; i < kJointCount; ++i) {
    const double initial = initial_value_for_joint(info_.joints[i]);
    hw_positions_[i] = std::clamp(initial, lower_limits_[i], upper_limits_[i]);
    hw_velocities_[i] = 0.0;
    hw_efforts_[i] = 0.0;
    hw_commands_pos_[i] = hw_positions_[i];
    last_sent_commands_[i] = hw_commands_pos_[i];
    feedback_positions_[i] = hw_positions_[i];
    feedback_velocities_[i] = 0.0;
    feedback_efforts_[i] = 0.0;
  }

  feedback_valid_ = false;
  feedback_online_.fill(false);
  command_ready_ = false;
  last_sent_valid_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName),
    "Configured WujiHand2Hardware: side=%s address='%s' sn='%s' mit_kp=%.3f mit_kd=%.3f "
    "effort_limit=%.3f read_feedback=%s",
    hand_side_.c_str(), device_address_.c_str(), serial_number_.c_str(), mit_kp_, mit_kd_,
    effort_limit_, read_feedback_ ? "true" : "false");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WujiHand2Hardware::on_activate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  shutting_down_ = false;
  command_ready_ = false;
  feedback_valid_ = false;
  feedback_online_.fill(false);

  // Official order: limit → MIT → enable → joint_command; subscribe after enable.
  if (!connect_device()) {
    safe_shutdown();
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!apply_mit_and_effort_limit()) {
    safe_shutdown();
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!enable_motors()) {
    safe_shutdown();
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (read_feedback_) {
    if (!start_state_subscription()) {
      safe_shutdown();
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (require_initial_feedback_) {
      const auto timeout = std::chrono::milliseconds(
        static_cast<int>(std::max(1000.0, enable_timeout_s_ * 1000.0)));
      if (!wait_initial_feedback(timeout)) {
        RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "Timed out waiting for joint_states");
        safe_shutdown();
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    initialize_state_from_feedback();
  } else {
    for (std::size_t i = 0; i < kJointCount; ++i) {
      hw_commands_pos_[i] = hw_positions_[i];
    }
  }

  if (!open_command_publisher()) {
    safe_shutdown();
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!send_joint_commands(hw_commands_pos_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "Initial hold send failed: %s", wuji_last_error());
    safe_shutdown();
    return hardware_interface::CallbackReturn::ERROR;
  }

  last_sent_commands_ = hw_commands_pos_;
  last_sent_valid_ = true;
  command_ready_ = true;
  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "Wuji Hand2 hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WujiHand2Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  safe_shutdown();
  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "Wuji Hand2 hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn WujiHand2Hardware::on_shutdown(
  const rclcpp_lifecycle::State & /* previous_state */)
{
  safe_shutdown();
  RCLCPP_INFO(rclcpp::get_logger(kLoggerName), "Wuji Hand2 hardware shut down");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
WujiHand2Hardware::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> interfaces;
  interfaces.reserve(kJointCount * 3);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
      joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
WujiHand2Hardware::on_export_command_interfaces()
{
  // Plan option A: export position command only (BJC / hand2.yaml).
  std::vector<hardware_interface::CommandInterface::SharedPtr> interfaces;
  interfaces.reserve(kJointCount);
  for (std::size_t i = 0; i < kJointCount; ++i) {
    interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_pos_[i]));
  }
  return interfaces;
}

hardware_interface::return_type WujiHand2Hardware::read(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (!read_feedback_) {
    hw_positions_ = hw_commands_pos_;
    hw_velocities_.fill(0.0);
    hw_efforts_.fill(0.0);
    return hardware_interface::return_type::OK;
  }

  std::lock_guard<std::mutex> lock(feedback_mutex_);
  if (feedback_valid_) {
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (!feedback_online_[i]) {
        continue;
      }
      hw_positions_[i] = feedback_positions_[i];
      hw_velocities_[i] = feedback_velocities_[i];
      hw_efforts_[i] = feedback_efforts_[i];
    }
  }
  // else: keep last hw_* values
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type WujiHand2Hardware::write(
  const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  if (!command_ready_.load()) {
    return hardware_interface::return_type::OK;
  }
  if (read_feedback_ && require_initial_feedback_ && !feedback_valid_) {
    return hardware_interface::return_type::OK;
  }

  std::array<double, kJointCount> clamped{};
  for (std::size_t i = 0; i < kJointCount; ++i) {
    clamped[i] = std::clamp(hw_commands_pos_[i], lower_limits_[i], upper_limits_[i]);
    hw_commands_pos_[i] = clamped[i];
  }

  if (!command_changed(clamped)) {
    return hardware_interface::return_type::OK;
  }

  if (!send_joint_commands(clamped)) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "joint_command send failed: %s", wuji_last_error());
    safe_shutdown();
    return hardware_interface::return_type::ERROR;
  }

  last_sent_commands_ = clamped;
  last_sent_valid_ = true;
  return hardware_interface::return_type::OK;
}

void WujiHand2Hardware::load_parameters()
{
  const auto & params = info_.hardware_parameters;
  auto get = [&](const std::string & key, const std::string & fallback) -> std::string {
    const auto it = params.find(key);
    return it == params.end() ? fallback : it->second;
  };

  hand_side_ = get("hand_side", "left");
  serial_number_ = get("serial_number", "");
  device_address_ = get("device_address", "");
  mit_kp_ = parse_double(get("mit_kp", ""), hand2::kDefaultMitKp);
  mit_kd_ = parse_double(get("mit_kd", ""), hand2::kDefaultMitKd);
  effort_limit_ = parse_double(get("effort_limit", ""), hand2::kDefaultEffortLimitA);
  read_feedback_ = parse_bool(get("read_feedback", ""), true);
  require_initial_feedback_ = parse_bool(get("require_initial_feedback", ""), true);
  command_deadband_ = parse_double(get("command_deadband", ""), 0.0);
  connect_timeout_ms_ = parse_int(get("connect_timeout_ms", ""), 5000);
  enable_timeout_s_ = parse_double(get("enable_timeout_s", ""), 5.0);

  if (hand_side_ != "left" && hand_side_ != "right") {
    RCLCPP_WARN(
      rclcpp::get_logger(kLoggerName),
      "Unknown hand_side='%s', defaulting to left", hand_side_.c_str());
    hand_side_ = "left";
  }
}

bool WujiHand2Hardware::validate_joint_interfaces() const
{
  for (const auto & joint : info_.joints) {
    const auto has_cmd_pos = std::any_of(
      joint.command_interfaces.begin(), joint.command_interfaces.end(),
      [](const hardware_interface::InterfaceInfo & iface) {
        return iface.name == hardware_interface::HW_IF_POSITION;
      });
    if (!has_cmd_pos) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' missing position command interface", joint.name.c_str());
      return false;
    }

    auto has_state = [&](const std::string & name) {
      return std::any_of(
        joint.state_interfaces.begin(), joint.state_interfaces.end(),
        [&](const hardware_interface::InterfaceInfo & iface) { return iface.name == name; });
    };
    if (!has_state(hardware_interface::HW_IF_POSITION) ||
        !has_state(hardware_interface::HW_IF_VELOCITY) ||
        !has_state(hardware_interface::HW_IF_EFFORT))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' must declare position+velocity+effort state interfaces",
        joint.name.c_str());
      return false;
    }
  }
  return true;
}

bool WujiHand2Hardware::build_joint_maps()
{
  ros_to_sdk_.fill(kJointCount);
  sdk_to_ros_.fill(kJointCount);

  for (std::size_t ros_i = 0; ros_i < joint_names_.size(); ++ros_i) {
    const std::size_t sdk_i = hand2::sdk_index_for_joint_name(joint_names_[ros_i]);
    if (sdk_i >= kJointCount) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Joint '%s' does not match assumed Hand2 name suffixes (see wuji_hand2_protocol.hpp)",
        joint_names_[ros_i].c_str());
      return false;
    }
    if (sdk_to_ros_[sdk_i] < kJointCount) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Duplicate mapping for SDK index %zu (joints '%s' and '%s')", sdk_i,
        joint_names_[sdk_to_ros_[sdk_i]].c_str(), joint_names_[ros_i].c_str());
      return false;
    }
    ros_to_sdk_[ros_i] = sdk_i;
    sdk_to_ros_[sdk_i] = ros_i;
  }

  for (std::size_t sdk_i = 0; sdk_i < kJointCount; ++sdk_i) {
    if (sdk_to_ros_[sdk_i] >= kJointCount) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "Missing ROS joint for assumed SDK index %zu (%s)", sdk_i,
        std::string(hand2::kJointNameSuffixes[sdk_i]).c_str());
      return false;
    }
  }
  return true;
}

bool WujiHand2Hardware::connect_device()
{
  if (device_ != nullptr) {
    return true;
  }

  const WujiStatus init_st = wuji_init(nullptr);
  if (init_st != WUJI_STATUS_OK) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName), "wuji_init failed: %s",
      status_message(init_st).c_str());
    return false;
  }

  WujiConnectOptions opts = wuji_connect_options_default();
  opts.timeout_ms = static_cast<uint32_t>(std::max(1, connect_timeout_ms_));
  opts.enable_bridge = true;

  WujiConnectTarget target{};
  std::string value_storage;

  if (!device_address_.empty()) {
    value_storage = device_address_;
    target.kind = WUJI_CONNECT_TARGET_KIND_ADDR;
    target.value = value_storage.c_str();
  } else if (!serial_number_.empty()) {
    value_storage = serial_number_;
    target.kind = WUJI_CONNECT_TARGET_KIND_SN;
    target.value = value_storage.c_str();
  } else {
    WujiDiscovered * found = nullptr;
    size_t found_n = 0;
    const WujiStatus scan_st = wuji_scan(&found, &found_n);
    if (scan_st != WUJI_STATUS_OK || found == nullptr || found_n == 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLoggerName),
        "wuji_scan found no devices: %s", status_message(scan_st).c_str());
      if (found != nullptr) {
        wuji_discovered_free(found, found_n);
      }
      return false;
    }

    const bool want_left = (hand_side_ == "left");
    std::vector<std::string> candidates;
    for (size_t i = 0; i < found_n; ++i) {
      if (found[i].device_id == WUJI_DEVICE_TYPE_WUJI_HAND_2) {
        candidates.emplace_back(found[i].serial_number);
      }
    }
    wuji_discovered_free(found, found_n);

    if (candidates.empty()) {
      RCLCPP_ERROR(rclcpp::get_logger(kLoggerName), "No WujiHand2 in scan results");
      return false;
    }

    for (const auto & sn : candidates) {
      value_storage = sn;
      target.kind = WUJI_CONNECT_TARGET_KIND_SN;
      target.value = value_storage.c_str();
      WujiDevice * trial = nullptr;
      const WujiStatus st = wuji_connect(&target, kDeviceAlias, &opts, &trial);
      if (st != WUJI_STATUS_OK || trial == nullptr) {
        continue;
      }
      WujiHandedness handedness = WUJI_HANDEDNESS_LEFT;
      const WujiStatus hs = wuji_hand_2_get_handedness(trial, &handedness);
      const bool is_left = (handedness == WUJI_HANDEDNESS_LEFT);
      const bool match =
        (hs != WUJI_STATUS_OK) ? (candidates.size() == 1) : (is_left == want_left);
      if (match) {
        device_ = trial;
        RCLCPP_INFO(
          rclcpp::get_logger(kLoggerName),
          "Connected WujiHand2 via scan SN=%s (hand_side=%s)", sn.c_str(),
          hand_side_.c_str());
        return true;
      }
      wuji_dev_disconnect(trial);
      wuji_dev_release(trial);
    }

    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName),
      "No scanned WujiHand2 matched hand_side=%s", hand_side_.c_str());
    return false;
  }

  WujiDevice * dev = nullptr;
  const WujiStatus st = wuji_connect(&target, kDeviceAlias, &opts, &dev);
  if (st != WUJI_STATUS_OK || dev == nullptr) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName), "wuji_connect failed: %s",
      status_message(st).c_str());
    return false;
  }
  device_ = dev;
  RCLCPP_INFO(
    rclcpp::get_logger(kLoggerName), "Connected WujiHand2 (kind=%u value=%s)",
    static_cast<unsigned>(target.kind), target.value);
  return true;
}

void WujiHand2Hardware::disconnect_device()
{
  if (device_ == nullptr) {
    return;
  }
  wuji_dev_disconnect(device_);
  wuji_dev_release(device_);
  device_ = nullptr;
}

bool WujiHand2Hardware::apply_mit_and_effort_limit()
{
  if (device_ == nullptr) {
    return false;
  }

  const WujiStatus lim_st =
    wuji_hand_2_set_all_effort_limit(device_, static_cast<float>(effort_limit_));
  if (lim_st != WUJI_STATUS_OK) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName), "set_all_effort_limit failed: %s",
      status_message(lim_st).c_str());
    return false;
  }

  std::array<float, kJointCount> kp{};
  std::array<float, kJointCount> kd{};
  kp.fill(static_cast<float>(mit_kp_));
  kd.fill(static_cast<float>(mit_kd_));
  const WujiStatus mit_st = wuji_hand_2_set_all_mit_params(device_, kp.data(), kd.data());
  if (mit_st != WUJI_STATUS_OK) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName), "set_all_mit_params failed: %s",
      status_message(mit_st).c_str());
    return false;
  }
  return true;
}

bool WujiHand2Hardware::enable_motors()
{
  if (device_ == nullptr) {
    return false;
  }
  const WujiStatus st = wuji_hand_2_enable(device_, nullptr);
  if (st != WUJI_STATUS_OK) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName), "enable failed: %s", status_message(st).c_str());
    return false;
  }
  // Brief settle; SDK has no separate wait_enabled helper for Hand2.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  return true;
}

bool WujiHand2Hardware::disable_motors()
{
  if (device_ == nullptr) {
    return true;
  }
  const WujiStatus st = wuji_hand_2_disable(device_, nullptr);
  if (st != WUJI_STATUS_OK) {
    RCLCPP_WARN(
      rclcpp::get_logger(kLoggerName), "disable failed: %s", status_message(st).c_str());
    return false;
  }
  return true;
}

void WujiHand2Hardware::safe_shutdown()
{
  if (shutting_down_) {
    // Allow re-entry only to finish partial teardown; keep idempotent.
  }
  shutting_down_ = true;
  command_ready_ = false;

  disable_motors();
  close_command_publisher();
  stop_state_subscription();
  disconnect_device();

  shutting_down_ = false;
}

bool WujiHand2Hardware::start_state_subscription()
{
  if (device_ == nullptr) {
    return false;
  }
  if (state_sub_ != nullptr) {
    return true;
  }

  WujiSub * sub = nullptr;
  const WujiStatus st =
    wuji_hand_2_subscribe_joint_states(device_, &WujiHand2Hardware::on_joint_states_callback, this, &sub);
  if (st != WUJI_STATUS_OK || sub == nullptr) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName), "subscribe_joint_states failed: %s",
      status_message(st).c_str());
    return false;
  }
  state_sub_ = sub;
  return true;
}

void WujiHand2Hardware::stop_state_subscription()
{
  if (state_sub_ == nullptr) {
    return;
  }
  // Must not call from the subscription callback thread.
  wuji_sub_close(state_sub_);
  state_sub_ = nullptr;
}

void WujiHand2Hardware::on_joint_states_callback(
  WujiFrameKind kind, const WujiJointStateFrame * frame, void * user_data)
{
  auto * self = static_cast<WujiHand2Hardware *>(user_data);
  if (self == nullptr) {
    return;
  }
  if (kind != WUJI_FRAME_KIND_OK || frame == nullptr || frame->joints == nullptr) {
    return;
  }

  std::lock_guard<std::mutex> lock(self->feedback_mutex_);
  self->feedback_online_.fill(false);
  for (size_t i = 0; i < frame->joints_len; ++i) {
    const WujiJointStateEntry & entry = frame->joints[i];
    const std::size_t sdk_i = hand2::flat_index_from_joint_states_nid(entry.nid);
    if (sdk_i >= kJointCount) {
      continue;
    }
    const std::size_t ros_i = self->sdk_to_ros_[sdk_i];
    if (ros_i >= kJointCount) {
      continue;
    }
    self->feedback_online_[ros_i] = true;
    self->feedback_positions_[ros_i] = static_cast<double>(entry.position);
    self->feedback_velocities_[ros_i] = static_cast<double>(entry.velocity);
    self->feedback_efforts_[ros_i] = static_cast<double>(entry.effort);
  }
  self->feedback_valid_ = true;
  self->feedback_cv_.notify_all();
}

bool WujiHand2Hardware::wait_initial_feedback(std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(feedback_mutex_);
  return feedback_cv_.wait_for(lock, timeout, [this] { return feedback_valid_; });
}

void WujiHand2Hardware::initialize_state_from_feedback()
{
  std::lock_guard<std::mutex> lock(feedback_mutex_);
  if (!feedback_valid_) {
    return;
  }
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (!feedback_online_[i]) {
      continue;
    }
    hw_positions_[i] = feedback_positions_[i];
    hw_velocities_[i] = feedback_velocities_[i];
    hw_efforts_[i] = feedback_efforts_[i];
    hw_commands_pos_[i] = hw_positions_[i];
  }
}

bool WujiHand2Hardware::open_command_publisher()
{
  if (device_ == nullptr) {
    return false;
  }
  if (command_pub_ != nullptr) {
    return true;
  }
  WujiJointCommandPublisher * pub = nullptr;
  const WujiStatus st = wuji_hand_2_joint_command_publish(device_, &pub);
  if (st != WUJI_STATUS_OK || pub == nullptr) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLoggerName), "joint_command_publish failed: %s",
      status_message(st).c_str());
    return false;
  }
  command_pub_ = pub;
  return true;
}

void WujiHand2Hardware::close_command_publisher()
{
  if (command_pub_ == nullptr) {
    return;
  }
  wuji_joint_command_publisher_close(command_pub_);
  command_pub_ = nullptr;
}

bool WujiHand2Hardware::send_joint_commands(
  const std::array<double, kJointCount> & ros_positions)
{
  if (command_pub_ == nullptr) {
    return false;
  }

  std::array<WujiJointCommand, kJointCount> cmds{};
  for (std::size_t ros_i = 0; ros_i < kJointCount; ++ros_i) {
    const std::size_t sdk_i = ros_to_sdk_[ros_i];
    cmds[sdk_i].position = static_cast<float>(ros_positions[ros_i]);
    cmds[sdk_i].velocity = 0.0f;
    cmds[sdk_i].effort = 0.0f;
  }

  const WujiStatus st = wuji_joint_command_publisher_send(command_pub_, cmds.data());
  return st == WUJI_STATUS_OK;
}

bool WujiHand2Hardware::command_changed(
  const std::array<double, kJointCount> & ros_positions) const
{
  if (!last_sent_valid_) {
    return true;
  }
  if (command_deadband_ <= 0.0) {
    return true;
  }
  for (std::size_t i = 0; i < kJointCount; ++i) {
    if (std::abs(ros_positions[i] - last_sent_commands_[i]) > command_deadband_) {
      return true;
    }
  }
  return false;
}

bool WujiHand2Hardware::parse_bool(const std::string & value, bool default_value)
{
  if (value.empty()) {
    return default_value;
  }
  if (value == "1" || value == "true" || value == "True" || value == "TRUE") {
    return true;
  }
  if (value == "0" || value == "false" || value == "False" || value == "FALSE") {
    return false;
  }
  return default_value;
}

int WujiHand2Hardware::parse_int(const std::string & value, int default_value)
{
  if (value.empty()) {
    return default_value;
  }
  try {
    return std::stoi(value);
  } catch (const std::exception &) {
    return default_value;
  }
}

double WujiHand2Hardware::parse_double(const std::string & value, double default_value)
{
  if (value.empty()) {
    return default_value;
  }
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    return default_value;
  }
}

double WujiHand2Hardware::initial_value_for_joint(
  const hardware_interface::ComponentInfo & joint)
{
  for (const auto & iface : joint.state_interfaces) {
    if (iface.name == hardware_interface::HW_IF_POSITION && !iface.initial_value.empty()) {
      try {
        return std::stod(iface.initial_value);
      } catch (const std::exception &) {
        return 0.0;
      }
    }
  }
  return 0.0;
}

}  // namespace wujihand2_ros2_control

PLUGINLIB_EXPORT_CLASS(
  wujihand2_ros2_control::WujiHand2Hardware, hardware_interface::SystemInterface)
