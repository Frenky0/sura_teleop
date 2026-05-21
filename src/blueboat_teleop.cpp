#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sura_msgs/msg/navigator.hpp"

class BlueboatTeleop : public rclcpp::Node
{
public:
  BlueboatTeleop()
  : Node("blueboat_teleop")
  {
    declare_parameter<std::string>("joy_topic", "/joy");
    declare_parameter<std::string>("cmd_vel_topic", "/sura/catamaran/cmd_vel");
    declare_parameter<std::string>("body_force_topic", "/sura/catamaran/body_force/command");
    declare_parameter<double>("publish_rate", 20.0);
    declare_parameter<double>("command_timeout", 0.5);

    declare_parameter<int>("axis_linear_x", 1);
    declare_parameter<int>("axis_angular_z", 3);

    declare_parameter<double>("scale_linear_x", 0.35);
    declare_parameter<double>("scale_angular_z", 0.3);
    declare_parameter<double>("turbo_scale_linear_x", 0.35);
    declare_parameter<double>("turbo_scale_angular_z", 0.3);
    declare_parameter<double>("scale_force_x", 12.0);
    declare_parameter<double>("scale_torque_z", 3.0);
    declare_parameter<double>("turbo_scale_force_x", 18.0);
    declare_parameter<double>("turbo_scale_torque_z", 4.5);
    declare_parameter<double>("force_filter_alpha", 0.25);
    declare_parameter<double>("max_force_rate_x", 24.0);
    declare_parameter<double>("max_torque_rate_z", 6.0);

    declare_parameter<int>("enable_button", -1);
    declare_parameter<int>("turbo_button", -1);
    declare_parameter<bool>("require_enable_button", false);

    declare_parameter<int>("mode_modifier_button", 5);
    declare_parameter<int>("alternate_mode_modifier_button", -1);
    declare_parameter<int>("body_force_button", 4);
    declare_parameter<int>("body_velocity_button", 2);
    declare_parameter<int>("body_position_button", 0);

    declare_parameter<std::string>(
      "controller_manager_name", "/sura/controller/controller_manager");
    declare_parameter<std::string>("body_force_controller_name", "body_force_controller");
    declare_parameter<std::string>("body_velocity_controller_name", "body_velocity_controller");
    declare_parameter<std::string>("body_position_controller_name", "body_position_controller");
    declare_parameter<std::string>("thruster_test_controller_name", "thruster_test_controller");
    declare_parameter<std::string>("navigator_topic", "/sura/catamaran/navigator_msg");
    declare_parameter<std::string>(
      "position_setpoint_topic", "/sura/catamaran/body_position/setpoint");

    declare_parameter<double>("axis_deadzone", 0.2);
    declare_parameter<bool>("invert_linear_x", false);
    declare_parameter<bool>("invert_angular_z", false);

    joy_topic_ = get_parameter("joy_topic").as_string();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    body_force_topic_ = get_parameter("body_force_topic").as_string();
    publish_rate_ = get_parameter("publish_rate").as_double();
    command_timeout_ = get_parameter("command_timeout").as_double();

    axis_linear_x_ = get_parameter("axis_linear_x").as_int();
    axis_angular_z_ = get_parameter("axis_angular_z").as_int();

    scale_linear_x_ = get_parameter("scale_linear_x").as_double();
    scale_angular_z_ = get_parameter("scale_angular_z").as_double();
    turbo_scale_linear_x_ = get_parameter("turbo_scale_linear_x").as_double();
    turbo_scale_angular_z_ = get_parameter("turbo_scale_angular_z").as_double();
    scale_force_x_ = get_parameter("scale_force_x").as_double();
    scale_torque_z_ = get_parameter("scale_torque_z").as_double();
    turbo_scale_force_x_ = get_parameter("turbo_scale_force_x").as_double();
    turbo_scale_torque_z_ = get_parameter("turbo_scale_torque_z").as_double();
    force_filter_alpha_ = clamp(get_parameter("force_filter_alpha").as_double(), 0.0, 1.0);
    max_force_rate_x_ = std::max(0.0, get_parameter("max_force_rate_x").as_double());
    max_torque_rate_z_ = std::max(0.0, get_parameter("max_torque_rate_z").as_double());

    enable_button_ = get_parameter("enable_button").as_int();
    turbo_button_ = get_parameter("turbo_button").as_int();
    require_enable_button_ = get_parameter("require_enable_button").as_bool();

    mode_modifier_button_ = get_parameter("mode_modifier_button").as_int();
    alternate_mode_modifier_button_ = get_parameter("alternate_mode_modifier_button").as_int();
    body_force_button_ = get_parameter("body_force_button").as_int();
    body_velocity_button_ = get_parameter("body_velocity_button").as_int();
    body_position_button_ = get_parameter("body_position_button").as_int();

    controller_manager_name_ = get_parameter("controller_manager_name").as_string();
    body_force_controller_name_ = get_parameter("body_force_controller_name").as_string();
    body_velocity_controller_name_ = get_parameter("body_velocity_controller_name").as_string();
    body_position_controller_name_ = get_parameter("body_position_controller_name").as_string();
    thruster_test_controller_name_ = get_parameter("thruster_test_controller_name").as_string();
    navigator_topic_ = get_parameter("navigator_topic").as_string();
    position_setpoint_topic_ = get_parameter("position_setpoint_topic").as_string();

    axis_deadzone_ = get_parameter("axis_deadzone").as_double();
    invert_linear_x_ = get_parameter("invert_linear_x").as_bool();
    invert_angular_z_ = get_parameter("invert_angular_z").as_bool();

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
    force_pub_ = create_publisher<geometry_msgs::msg::Wrench>(body_force_topic_, 10);
    position_setpoint_pub_ =
      create_publisher<geometry_msgs::msg::PoseStamped>(position_setpoint_topic_, 10);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_, 10, std::bind(&BlueboatTeleop::joyCallback, this, std::placeholders::_1));
    navigator_sub_ = create_subscription<sura_msgs::msg::Navigator>(
      navigator_topic_, 10,
      std::bind(&BlueboatTeleop::navigatorCallback, this, std::placeholders::_1));

    switch_client_ = create_client<controller_manager_msgs::srv::SwitchController>(
      controller_manager_name_ + "/switch_controller");
    list_controllers_client_ = create_client<controller_manager_msgs::srv::ListControllers>(
      controller_manager_name_ + "/list_controllers");

    last_publish_time_ = get_clock()->now();
    const double timer_period = publish_rate_ > 0.0 ? 1.0 / publish_rate_ : 0.05;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(timer_period),
      std::bind(&BlueboatTeleop::publishCommand, this));
    controller_state_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&BlueboatTeleop::refreshControllerState, this));

    RCLCPP_INFO(get_logger(), "Blueboat teleop ready");
  }

private:
  using JoyMsg = sensor_msgs::msg::Joy;
  using TwistMsg = geometry_msgs::msg::Twist;
  using WrenchMsg = geometry_msgs::msg::Wrench;
  using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
  using NavigatorMsg = sura_msgs::msg::Navigator;
  using ListControllersSrv = controller_manager_msgs::srv::ListControllers;
  using SwitchControllerSrv = controller_manager_msgs::srv::SwitchController;

  void navigatorCallback(const NavigatorMsg::SharedPtr msg) {last_navigator_msg_ = msg;}

  void joyCallback(const JoyMsg::SharedPtr msg)
  {
    last_joy_msg_ = msg;
    last_joy_time_ = get_clock()->now();
    zero_twist_sent_ = false;
    zero_wrench_sent_ = false;
    handleModeSwitchShortcuts(msg->buttons);
  }

  void handleModeSwitchShortcuts(const std::vector<int32_t> & buttons)
  {
    const bool modifier_pressed = modeModifierPressed(buttons);
    const bool force_combo_pressed = modifier_pressed && buttonPressed(buttons, body_force_button_);
    const bool velocity_combo_pressed =
      modifier_pressed && buttonPressed(buttons, body_velocity_button_);
    const bool position_combo_pressed =
      modifier_pressed && buttonPressed(buttons, body_position_button_);

    if (force_combo_pressed && !prev_force_combo_pressed_) {
      if (active_mode_ == "force") {
        switchMode({}, {body_force_controller_name_, thruster_test_controller_name_},
          "body force disabled", "none");
      } else if (active_mode_ == "position") {
        switchMode({}, {
            body_position_controller_name_,
            body_velocity_controller_name_,
            body_force_controller_name_,
            thruster_test_controller_name_},
          "body position, velocity and force disabled", "none");
      } else if (active_mode_ == "velocity") {
        switchMode({}, {
            body_velocity_controller_name_,
            body_force_controller_name_,
            thruster_test_controller_name_},
          "body force and velocity disabled", "none");
      } else {
        switchMode(
          {body_force_controller_name_},
          {body_velocity_controller_name_, thruster_test_controller_name_},
          "body force", "force");
      }
    } else if (velocity_combo_pressed && !prev_velocity_combo_pressed_) {
      if (active_mode_ == "velocity") {
        switchMode({}, {body_velocity_controller_name_}, "body velocity disabled", "force");
      } else {
        switchModeSequence({
            {
              {body_force_controller_name_},
              {body_position_controller_name_, thruster_test_controller_name_},
              "body force prerequisite",
              std::nullopt
            },
            {
              {body_velocity_controller_name_},
              {},
              "body velocity",
              std::optional<std::string>("velocity")
            }
          });
      }
    } else if (position_combo_pressed && !prev_position_combo_pressed_) {
      if (active_mode_ == "position") {
        switchMode({}, {body_position_controller_name_}, "body position disabled", "velocity");
      } else {
        if (!publishHoldSetpoint()) {
          RCLCPP_WARN(
            get_logger(), "Cannot enable body position mode without navigator feedback yet");
        } else {
          switchModeSequence({
              {
                {body_force_controller_name_},
                {thruster_test_controller_name_},
                "body force prerequisite",
                std::nullopt
              },
              {
                {body_velocity_controller_name_},
                {},
                "body velocity prerequisite",
                std::nullopt
              },
              {
                {body_position_controller_name_},
                {},
                "body position",
                std::optional<std::string>("position")
              }
            });
        }
      }
    }

    prev_force_combo_pressed_ = force_combo_pressed;
    prev_velocity_combo_pressed_ = velocity_combo_pressed;
    prev_position_combo_pressed_ = position_combo_pressed;
  }

  struct SwitchStep
  {
    std::vector<std::string> start;
    std::vector<std::string> stop;
    std::string label;
    std::optional<std::string> target_mode;
  };

  bool modeModifierPressed(const std::vector<int32_t> & buttons) const
  {
    return buttonPressed(buttons, mode_modifier_button_) ||
           buttonPressed(buttons, alternate_mode_modifier_button_);
  }

  bool publishHoldSetpoint()
  {
    if (!last_navigator_msg_) {
      return false;
    }

    PoseStampedMsg setpoint;
    setpoint.header.stamp = get_clock()->now();
    setpoint.header.frame_id = "map";
    setpoint.pose = last_navigator_msg_->position;
    position_setpoint_pub_->publish(setpoint);
    return true;
  }

  void switchModeSequence(const std::vector<SwitchStep> & steps)
  {
    if (pending_switch_ || steps.empty()) {
      return;
    }

    pending_switch_steps_.assign(steps.begin() + 1, steps.end());
    const auto & first_step = steps.front();
    switchMode(first_step.start, first_step.stop, first_step.label, first_step.target_mode);
  }

  void switchMode(
    const std::vector<std::string> & start,
    const std::vector<std::string> & stop,
    const std::string & label,
    const std::optional<std::string> & target_mode)
  {
    if (pending_switch_) {
      return;
    }

    if (!switch_client_->wait_for_service(std::chrono::seconds(0))) {
      RCLCPP_WARN(get_logger(), "switch_controller service not available");
      return;
    }

    pending_switch_ = true;
    publishStopCommands(0.0, true);

    auto request = std::make_shared<SwitchControllerSrv::Request>();
    request->activate_controllers = start;
    request->deactivate_controllers = stop;
    request->strictness = SwitchControllerSrv::Request::BEST_EFFORT;
    request->activate_asap = true;

    auto future = switch_client_->async_send_request(
      request,
      [this, label, target_mode](rclcpp::Client<SwitchControllerSrv>::SharedFuture response_future) {
        handleSwitchResult(response_future, label, target_mode);
      });
    (void)future;
  }

  void handleSwitchResult(
    rclcpp::Client<SwitchControllerSrv>::SharedFuture future,
    const std::string & label,
    const std::optional<std::string> & target_mode)
  {
    pending_switch_ = false;
    auto response = future.get();
    if (response && response->ok) {
      if (target_mode) {
        active_mode_ = *target_mode;
      }
      RCLCPP_INFO(get_logger(), "Controller switch complete: %s", label.c_str());
      if (!pending_switch_steps_.empty()) {
        const auto next_step = pending_switch_steps_.front();
        pending_switch_steps_.erase(pending_switch_steps_.begin());
        switchMode(next_step.start, next_step.stop, next_step.label, next_step.target_mode);
      }
    } else {
      pending_switch_steps_.clear();
      RCLCPP_WARN(
        get_logger(), "Controller switch to %s mode was rejected", label.c_str());
    }
  }

  void refreshControllerState()
  {
    if (pending_switch_) {
      return;
    }
    if (!list_controllers_client_->wait_for_service(std::chrono::seconds(0))) {
      return;
    }

    auto request = std::make_shared<ListControllersSrv::Request>();
    auto future = list_controllers_client_->async_send_request(
      request,
      [this](rclcpp::Client<ListControllersSrv>::SharedFuture response_future) {
        handleControllerState(response_future);
      });
    (void)future;
  }

  void handleControllerState(rclcpp::Client<ListControllersSrv>::SharedFuture future)
  {
    if (pending_switch_) {
      return;
    }

    auto response = future.get();
    std::vector<std::string> active;
    bool body_force_chained = false;
    for (const auto & controller : response->controller) {
      if (controller.state == "active") {
        active.push_back(controller.name);
      }
      if (controller.name == body_force_controller_name_) {
        body_force_chained = controller.is_chained;
      }
    }

    std::string detected_mode = "none";
    if (std::find(active.begin(), active.end(), body_position_controller_name_) != active.end()) {
      detected_mode = "position";
    } else if (
      std::find(active.begin(), active.end(), body_velocity_controller_name_) != active.end())
    {
      detected_mode = "velocity";
    } else if (
      std::find(active.begin(), active.end(), body_force_controller_name_) != active.end() &&
      !body_force_chained)
    {
      detected_mode = "force";
    }

    if (detected_mode != active_mode_) {
      active_mode_ = detected_mode;
      resetFilteredForce();
      RCLCPP_INFO(get_logger(), "Detected controller mode: %s", active_mode_.c_str());
    }
  }

  void publishCommand()
  {
    const auto now = get_clock()->now();
    const double dt = std::max(0.0, (now - last_publish_time_).seconds());
    last_publish_time_ = now;

    if (!last_joy_msg_) {
      if (!warned_waiting_for_joy_) {
        RCLCPP_WARN(
          get_logger(),
          "No Joy messages received yet on %s. Check joy_node and /dev/input.",
          joy_topic_.c_str());
        warned_waiting_for_joy_ = true;
      }
      publishStopCommands(dt, false);
      return;
    }

    if ((get_clock()->now() - last_joy_time_) > rclcpp::Duration::from_seconds(command_timeout_)) {
      publishStopCommands(dt, false);
      return;
    }

    if (pending_switch_ || !teleopEnabled(last_joy_msg_->buttons)) {
      publishStopCommands(dt, false);
      return;
    }

    double linear_x = axisValue(last_joy_msg_->axes, axis_linear_x_);
    double angular_z = axisValue(last_joy_msg_->axes, axis_angular_z_);

    if (invert_linear_x_) {
      linear_x *= -1.0;
    }
    if (invert_angular_z_) {
      angular_z *= -1.0;
    }

    if (active_mode_ == "force") {
      publishForceCommand(linear_x, angular_z, dt);
    } else if (active_mode_ == "velocity") {
      publishVelocityCommand(linear_x, angular_z);
      publishWrenchIfChanged(WrenchMsg{});
    } else {
      publishStopCommands(dt, false);
    }
  }

  void publishVelocityCommand(double linear_x, double angular_z)
  {
    double linear_scale = scale_linear_x_;
    double angular_scale = scale_angular_z_;

    if (buttonPressed(last_joy_msg_->buttons, turbo_button_)) {
      linear_scale = turbo_scale_linear_x_;
      angular_scale = turbo_scale_angular_z_;
    }

    TwistMsg twist;
    twist.linear.x = linear_x * linear_scale;
    twist.angular.z = angular_z * angular_scale;
    publishTwistIfChanged(twist);
  }

  void publishForceCommand(double linear_x, double angular_z, double dt)
  {
    double force_scale = scale_force_x_;
    double torque_scale = scale_torque_z_;

    if (buttonPressed(last_joy_msg_->buttons, turbo_button_)) {
      force_scale = turbo_scale_force_x_;
      torque_scale = turbo_scale_torque_z_;
    }

    const double target_force_x = linear_x * force_scale;
    const double target_torque_z = angular_z * torque_scale;

    filtered_force_x_ = filteredValue(
      filtered_force_x_, target_force_x, force_filter_alpha_, max_force_rate_x_, dt);
    filtered_torque_z_ = filteredValue(
      filtered_torque_z_, target_torque_z, force_filter_alpha_, max_torque_rate_z_, dt);

    WrenchMsg wrench;
    wrench.force.x = filtered_force_x_;
    wrench.torque.z = filtered_torque_z_;

    publishTwistIfChanged(TwistMsg{});
    publishWrenchIfChanged(wrench);
  }

  bool teleopEnabled(const std::vector<int32_t> & buttons) const
  {
    if (!require_enable_button_) {
      return true;
    }
    return buttonPressed(buttons, enable_button_);
  }

  double axisValue(const std::vector<float> & axes, int index) const
  {
    if (index < 0 || static_cast<size_t>(index) >= axes.size()) {
      return 0.0;
    }
    const double value = static_cast<double>(axes[static_cast<size_t>(index)]);
    if (std::abs(value) < axis_deadzone_) {
      return 0.0;
    }
    return value;
  }

  static bool buttonPressed(const std::vector<int32_t> & buttons, int index)
  {
    if (index < 0 || static_cast<size_t>(index) >= buttons.size()) {
      return false;
    }
    return buttons[static_cast<size_t>(index)] != 0;
  }

  void publishStopCommands(double dt, bool immediate)
  {
    if (immediate) {
      filtered_force_x_ = 0.0;
      filtered_torque_z_ = 0.0;
    } else {
      filtered_force_x_ = filteredValue(
        filtered_force_x_, 0.0, force_filter_alpha_, max_force_rate_x_, dt);
      filtered_torque_z_ = filteredValue(
        filtered_torque_z_, 0.0, force_filter_alpha_, max_torque_rate_z_, dt);
    }

    WrenchMsg wrench;
    wrench.force.x = filtered_force_x_;
    wrench.torque.z = filtered_torque_z_;

    publishTwistIfChanged(TwistMsg{});
    publishWrenchIfChanged(wrench);
  }

  void resetFilteredForce()
  {
    filtered_force_x_ = 0.0;
    filtered_torque_z_ = 0.0;
    last_published_wrench_ = WrenchMsg{};
    zero_wrench_sent_ = false;
  }

  void publishTwistIfChanged(const TwistMsg & twist)
  {
    const bool is_zero = isZeroTwist(twist);
    if (twistEqual(last_published_twist_, twist) && is_zero && zero_twist_sent_) {
      return;
    }
    cmd_pub_->publish(twist);
    last_published_twist_ = twist;
    zero_twist_sent_ = is_zero;
  }

  void publishWrenchIfChanged(const WrenchMsg & wrench)
  {
    const bool is_zero = isZeroWrench(wrench);
    if (wrenchEqual(last_published_wrench_, wrench) && is_zero && zero_wrench_sent_) {
      return;
    }
    force_pub_->publish(wrench);
    last_published_wrench_ = wrench;
    zero_wrench_sent_ = is_zero;
  }

  static bool twistEqual(const TwistMsg & lhs, const TwistMsg & rhs, double eps = 1e-9)
  {
    return
      std::abs(lhs.linear.x - rhs.linear.x) < eps &&
      std::abs(lhs.linear.y - rhs.linear.y) < eps &&
      std::abs(lhs.linear.z - rhs.linear.z) < eps &&
      std::abs(lhs.angular.x - rhs.angular.x) < eps &&
      std::abs(lhs.angular.y - rhs.angular.y) < eps &&
      std::abs(lhs.angular.z - rhs.angular.z) < eps;
  }

  static bool wrenchEqual(const WrenchMsg & lhs, const WrenchMsg & rhs, double eps = 1e-9)
  {
    return
      std::abs(lhs.force.x - rhs.force.x) < eps &&
      std::abs(lhs.force.y - rhs.force.y) < eps &&
      std::abs(lhs.force.z - rhs.force.z) < eps &&
      std::abs(lhs.torque.x - rhs.torque.x) < eps &&
      std::abs(lhs.torque.y - rhs.torque.y) < eps &&
      std::abs(lhs.torque.z - rhs.torque.z) < eps;
  }

  static bool isZeroTwist(const TwistMsg & twist, double eps = 1e-9)
  {
    return
      std::abs(twist.linear.x) < eps &&
      std::abs(twist.linear.y) < eps &&
      std::abs(twist.linear.z) < eps &&
      std::abs(twist.angular.x) < eps &&
      std::abs(twist.angular.y) < eps &&
      std::abs(twist.angular.z) < eps;
  }

  static bool isZeroWrench(const WrenchMsg & wrench, double eps = 1e-9)
  {
    return
      std::abs(wrench.force.x) < eps &&
      std::abs(wrench.force.y) < eps &&
      std::abs(wrench.force.z) < eps &&
      std::abs(wrench.torque.x) < eps &&
      std::abs(wrench.torque.y) < eps &&
      std::abs(wrench.torque.z) < eps;
  }

  static double filteredValue(
    double current,
    double target,
    double alpha,
    double max_rate,
    double dt)
  {
    double filtered = current + alpha * (target - current);
    if (max_rate > 0.0 && dt > 0.0) {
      const double max_delta = max_rate * dt;
      filtered = current + clamp(filtered - current, -max_delta, max_delta);
    }
    return filtered;
  }

  static double clamp(double value, double minimum, double maximum)
  {
    return std::max(minimum, std::min(maximum, value));
  }

  std::string joy_topic_;
  std::string cmd_vel_topic_;
  std::string body_force_topic_;
  double publish_rate_{20.0};
  double command_timeout_{0.5};
  int axis_linear_x_{1};
  int axis_angular_z_{3};
  double scale_linear_x_{0.35};
  double scale_angular_z_{0.3};
  double turbo_scale_linear_x_{0.35};
  double turbo_scale_angular_z_{0.3};
  double scale_force_x_{12.0};
  double scale_torque_z_{3.0};
  double turbo_scale_force_x_{18.0};
  double turbo_scale_torque_z_{4.5};
  double force_filter_alpha_{0.25};
  double max_force_rate_x_{24.0};
  double max_torque_rate_z_{6.0};
  int enable_button_{-1};
  int turbo_button_{-1};
  bool require_enable_button_{false};
  int mode_modifier_button_{5};
  int alternate_mode_modifier_button_{-1};
  int body_force_button_{4};
  int body_velocity_button_{2};
  int body_position_button_{0};
  std::string controller_manager_name_;
  std::string body_force_controller_name_;
  std::string body_velocity_controller_name_;
  std::string body_position_controller_name_;
  std::string thruster_test_controller_name_;
  std::string navigator_topic_;
  std::string position_setpoint_topic_;
  double axis_deadzone_{0.2};
  bool invert_linear_x_{false};
  bool invert_angular_z_{false};

  rclcpp::Publisher<TwistMsg>::SharedPtr cmd_pub_;
  rclcpp::Publisher<WrenchMsg>::SharedPtr force_pub_;
  rclcpp::Publisher<PoseStampedMsg>::SharedPtr position_setpoint_pub_;
  rclcpp::Subscription<JoyMsg>::SharedPtr joy_sub_;
  rclcpp::Subscription<NavigatorMsg>::SharedPtr navigator_sub_;
  rclcpp::Client<SwitchControllerSrv>::SharedPtr switch_client_;
  rclcpp::Client<ListControllersSrv>::SharedPtr list_controllers_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr controller_state_timer_;

  JoyMsg::SharedPtr last_joy_msg_;
  NavigatorMsg::SharedPtr last_navigator_msg_;
  rclcpp::Time last_joy_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};
  TwistMsg last_published_twist_;
  WrenchMsg last_published_wrench_;
  double filtered_force_x_{0.0};
  double filtered_torque_z_{0.0};
  bool zero_twist_sent_{false};
  bool zero_wrench_sent_{false};
  bool pending_switch_{false};
  std::string active_mode_{"none"};
  bool prev_force_combo_pressed_{false};
  bool prev_velocity_combo_pressed_{false};
  bool prev_position_combo_pressed_{false};
  bool warned_waiting_for_joy_{false};
  std::vector<SwitchStep> pending_switch_steps_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BlueboatTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
