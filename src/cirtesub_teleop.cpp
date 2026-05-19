#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

class CirtesubTeleop : public rclcpp::Node
{
public:
  CirtesubTeleop()
  : Node("cirtesub_teleop")
  {
    declare_parameter<double>("rate", 20.0);
    declare_parameter<std::string>("joy_topic", "/joy");
    declare_parameter<std::string>(
      "controller_switch_service", "/cirtesub/controller/controller_manager/switch_controller");
    declare_parameter<std::string>(
      "controller_list_service", "/cirtesub/controller/controller_manager/list_controllers");
    declare_parameter<std::string>("body_force_controller.name", "body_force");
    declare_parameter<std::string>(
      "body_force_controller.command_topic",
      "/cirtesub/controller/body_force/command");
    declare_parameter<double>("body_force_controller.feedforward_gain_x", 20.0);
    declare_parameter<double>("body_force_controller.feedforward_gain_y", 20.0);
    declare_parameter<double>("body_force_controller.feedforward_gain_z", 90.0);
    declare_parameter<double>("body_force_controller.feedforward_gain_roll", 20.0);
    declare_parameter<double>("body_force_controller.feedforward_gain_pitch", 20.0);
    declare_parameter<double>("body_force_controller.feedforward_gain_yaw", 1.0);
    declare_parameter<std::string>("body_velocity_controller.name", "body_velocity");
    declare_parameter<std::string>(
      "body_velocity_controller.setpoint_topic",
      "/cirtesub/controller/body_velocity/setpoint");
    declare_parameter<std::string>("position_hold_controller.name", "position_hold");
    declare_parameter<std::string>(
      "position_hold_controller.feedforward_topic",
      "/cirtesub/controller/position_hold/feedforward");
    declare_parameter<std::string>("stabilize_controller.name", "stabilize");
    declare_parameter<std::string>(
      "stabilize_controller.feedforward_topic",
      "/cirtesub/controller/stabilize/feedforward");
    declare_parameter<double>("stabilize_controller.feedforward_gain_x", 20.0);
    declare_parameter<double>("stabilize_controller.feedforward_gain_y", 20.0);
    declare_parameter<double>("stabilize_controller.feedforward_gain_z", 90.0);
    declare_parameter<double>("stabilize_controller.feedforward_gain_roll", 20.0);
    declare_parameter<double>("stabilize_controller.feedforward_gain_pitch", 20.0);
    declare_parameter<double>("stabilize_controller.feedforward_gain_yaw", 1.0);
    declare_parameter<std::string>(
      "stabilize_controller.enable_roll_pitch_service",
      "/cirtesub/controller/stabilize/enable_roll_pitch");
    declare_parameter<std::string>(
      "stabilize_controller.disable_roll_pitch_service",
      "/cirtesub/controller/stabilize/disable_roll_pitch");
    declare_parameter<std::string>("depth_hold_controller.name", "depth_hold");
    declare_parameter<std::string>(
      "depth_hold_controller.feedforward_topic",
      "/cirtesub/controller/depth_hold/feedforward");
    declare_parameter<double>("depth_hold_controller.feedforward_gain_x", 20.0);
    declare_parameter<double>("depth_hold_controller.feedforward_gain_y", 20.0);
    declare_parameter<double>("depth_hold_controller.feedforward_gain_z", 90.0);
    declare_parameter<double>("depth_hold_controller.feedforward_gain_roll", 20.0);
    declare_parameter<double>("depth_hold_controller.feedforward_gain_pitch", 20.0);
    declare_parameter<double>("depth_hold_controller.feedforward_gain_yaw", 1.0);
    declare_parameter<std::string>(
      "depth_hold_controller.enable_roll_pitch_service",
      "/cirtesub/controller/depth_hold/enable_roll_pitch");
    declare_parameter<std::string>(
      "depth_hold_controller.disable_roll_pitch_service",
      "/cirtesub/controller/depth_hold/disable_roll_pitch");
    declare_parameter<int>("buttons.a", 0);
    declare_parameter<int>("buttons.b", 1);
    declare_parameter<int>("buttons.x", 2);
    declare_parameter<int>("buttons.lb", 4);
    declare_parameter<int>("buttons.rb", 5);
    declare_parameter<int>("buttons.y", 3);
    declare_parameter<int>("buttons.left_stick", 9);
    declare_parameter<int>("buttons.right_stick", 10);
    declare_parameter<int>("axes.hat_horizontal", 6);
    declare_parameter<int>("axes.hat_vertical", 7);
    declare_parameter<int>("axes.surge", 1);
    declare_parameter<int>("axes.sway", 0);
    declare_parameter<int>("axes.yaw", 3);
    declare_parameter<int>("axes.heave", 4);
    declare_parameter<int>("axes.roll", 3);
    declare_parameter<int>("axes.pitch", 4);
    declare_parameter<std::string>(
      "alpha_left_forward_velocity_controller.name",
      "alpha_left_forward_velocity_controller");
    declare_parameter<std::string>(
      "alpha_right_forward_velocity_controller.name",
      "alpha_right_forward_velocity_controller");
    declare_parameter<std::string>(
      "alpha_left_forward_velocity_controller.command_topic",
      "/cirtesub/controller/alpha_left_forward_velocity_controller/commands");
    declare_parameter<std::string>(
      "alpha_right_forward_velocity_controller.command_topic",
      "/cirtesub/controller/alpha_right_forward_velocity_controller/commands");
    declare_parameter<double>("alpha_forward_command_rate", 10.0);
    declare_parameter<double>("scales.surge", 1.0);
    declare_parameter<double>("scales.sway", 1.0);
    declare_parameter<double>("scales.yaw", 1.0);
    declare_parameter<double>("scales.heave", 1.0);
    declare_parameter<double>("scales.roll", 1.0);
    declare_parameter<double>("scales.pitch", 1.0);
    declare_parameter<double>("deadzone", 0.05);
    // Declare parameters for any additional controllers and buttons as needed
    declare_parameter<std::string>("newton_gripper_controller.command_topic","/cirtesub/controller/newton_gripper_controller/commands");
    declare_parameter<std::string>("servo_zip_controller.command_topic","/cirtesub/controller/servo_zip_controller/commands");
    declare_parameter<std::string>("servo_camera_controller.command_topic","/cirtesub/controller/servo_camera_controller/commands");
    declare_parameter<double>("servo_command_rate", 20.0);
  declare_parameter<double>("servo_step", 0.1);





    rate_ = get_parameter("rate").as_double();
    joy_topic_ = get_parameter("joy_topic").as_string();
    controller_switch_service_ = get_parameter("controller_switch_service").as_string();
    controller_list_service_ = get_parameter("controller_list_service").as_string();
    body_force_controller_name_ = get_parameter("body_force_controller.name").as_string();
    body_force_command_topic_ = get_parameter("body_force_controller.command_topic").as_string();
    body_force_feedforward_gain_x_ =
      get_parameter("body_force_controller.feedforward_gain_x").as_double();
    body_force_feedforward_gain_y_ =
      get_parameter("body_force_controller.feedforward_gain_y").as_double();
    body_force_feedforward_gain_z_ =
      get_parameter("body_force_controller.feedforward_gain_z").as_double();
    body_force_feedforward_gain_roll_ =
      get_parameter("body_force_controller.feedforward_gain_roll").as_double();
    body_force_feedforward_gain_pitch_ =
      get_parameter("body_force_controller.feedforward_gain_pitch").as_double();
    body_force_feedforward_gain_yaw_ =
      get_parameter("body_force_controller.feedforward_gain_yaw").as_double();
    body_velocity_controller_name_ = get_parameter("body_velocity_controller.name").as_string();
    body_velocity_setpoint_topic_ =
      get_parameter("body_velocity_controller.setpoint_topic").as_string();
    position_hold_controller_name_ = get_parameter("position_hold_controller.name").as_string();
    position_hold_feedforward_topic_ =
      get_parameter("position_hold_controller.feedforward_topic").as_string();
    stabilize_controller_name_ = get_parameter("stabilize_controller.name").as_string();
    stabilize_feedforward_topic_ =
      get_parameter("stabilize_controller.feedforward_topic").as_string();
    stabilize_feedforward_gain_x_ =
      get_parameter("stabilize_controller.feedforward_gain_x").as_double();
    stabilize_feedforward_gain_y_ =
      get_parameter("stabilize_controller.feedforward_gain_y").as_double();
    stabilize_feedforward_gain_z_ =
      get_parameter("stabilize_controller.feedforward_gain_z").as_double();
    stabilize_feedforward_gain_roll_ =
      get_parameter("stabilize_controller.feedforward_gain_roll").as_double();
    stabilize_feedforward_gain_pitch_ =
      get_parameter("stabilize_controller.feedforward_gain_pitch").as_double();
    stabilize_feedforward_gain_yaw_ =
      get_parameter("stabilize_controller.feedforward_gain_yaw").as_double();
    stabilize_enable_roll_pitch_service_name_ =
      get_parameter("stabilize_controller.enable_roll_pitch_service").as_string();
    stabilize_disable_roll_pitch_service_name_ =
      get_parameter("stabilize_controller.disable_roll_pitch_service").as_string();
    depth_hold_controller_name_ = get_parameter("depth_hold_controller.name").as_string();
    depth_hold_feedforward_topic_ =
      get_parameter("depth_hold_controller.feedforward_topic").as_string();
    depth_hold_feedforward_gain_x_ =
      get_parameter("depth_hold_controller.feedforward_gain_x").as_double();
    depth_hold_feedforward_gain_y_ =
      get_parameter("depth_hold_controller.feedforward_gain_y").as_double();
    depth_hold_feedforward_gain_z_ =
      get_parameter("depth_hold_controller.feedforward_gain_z").as_double();
    depth_hold_feedforward_gain_roll_ =
      get_parameter("depth_hold_controller.feedforward_gain_roll").as_double();
    depth_hold_feedforward_gain_pitch_ =
      get_parameter("depth_hold_controller.feedforward_gain_pitch").as_double();
    depth_hold_feedforward_gain_yaw_ =
      get_parameter("depth_hold_controller.feedforward_gain_yaw").as_double();
    depth_hold_enable_roll_pitch_service_name_ =
      get_parameter("depth_hold_controller.enable_roll_pitch_service").as_string();
    depth_hold_disable_roll_pitch_service_name_ =
      get_parameter("depth_hold_controller.disable_roll_pitch_service").as_string();
    a_button_ = get_parameter("buttons.a").as_int();
    b_button_ = get_parameter("buttons.b").as_int();
    x_button_ = get_parameter("buttons.x").as_int();
    lb_button_ = get_parameter("buttons.lb").as_int();
    rb_button_ = get_parameter("buttons.rb").as_int();
    y_button_ = get_parameter("buttons.y").as_int();
    left_stick_button_ = get_parameter("buttons.left_stick").as_int();
    right_stick_button_ = get_parameter("buttons.right_stick").as_int();
    hat_horizontal_axis_ = get_parameter("axes.hat_horizontal").as_int();
    hat_vertical_axis_ = get_parameter("axes.hat_vertical").as_int();
    surge_axis_ = get_parameter("axes.surge").as_int();
    sway_axis_ = get_parameter("axes.sway").as_int();
    yaw_axis_ = get_parameter("axes.yaw").as_int();
    heave_axis_ = get_parameter("axes.heave").as_int();
    roll_axis_ = get_parameter("axes.roll").as_int();
    pitch_axis_ = get_parameter("axes.pitch").as_int();
    alpha_left_forward_velocity_controller_name_ =
      get_parameter("alpha_left_forward_velocity_controller.name").as_string();
    alpha_right_forward_velocity_controller_name_ =
      get_parameter("alpha_right_forward_velocity_controller.name").as_string();
    alpha_left_forward_velocity_command_topic_ =
      get_parameter("alpha_left_forward_velocity_controller.command_topic").as_string();
    alpha_right_forward_velocity_command_topic_ =
      get_parameter("alpha_right_forward_velocity_controller.command_topic").as_string();
    alpha_forward_command_rate_ = get_parameter("alpha_forward_command_rate").as_double();
    surge_scale_ = get_parameter("scales.surge").as_double();
    sway_scale_ = get_parameter("scales.sway").as_double();
    yaw_scale_ = get_parameter("scales.yaw").as_double();
    heave_scale_ = get_parameter("scales.heave").as_double();
    roll_scale_ = get_parameter("scales.roll").as_double();
    pitch_scale_ = get_parameter("scales.pitch").as_double();
    deadzone_ = std::max(0.0, get_parameter("deadzone").as_double());
    
    // Load any additional parameters for new controllers and buttons as needed
    newton_gripper_command_topic_ = get_parameter("newton_gripper_controller.command_topic").as_string();
    servo_zip_command_topic_ =  get_parameter("servo_zip_controller.command_topic").as_string();
    servo_camera_command_topic_ = get_parameter("servo_camera_controller.command_topic").as_string();
    servo_command_rate_ = get_parameter("servo_command_rate").as_double();
    servo_step_ = get_parameter("servo_step").as_double();
    //

    if (rate_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "Invalid rate %.3f Hz, using 20.0 Hz.", rate_);
      rate_ = 20.0;
    }

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      joy_topic_,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&CirtesubTeleop::joyCallback, this, std::placeholders::_1));

    updateWrenchPublisher(stabilize_feedforward_topic_);

    switch_controller_client_ =
      create_client<controller_manager_msgs::srv::SwitchController>(controller_switch_service_);
    list_controllers_client_ =
      create_client<controller_manager_msgs::srv::ListControllers>(controller_list_service_);
    stabilize_enable_roll_pitch_client_ =
      create_client<std_srvs::srv::Trigger>(stabilize_enable_roll_pitch_service_name_);
    stabilize_disable_roll_pitch_client_ =
      create_client<std_srvs::srv::Trigger>(stabilize_disable_roll_pitch_service_name_);
    depth_hold_enable_roll_pitch_client_ =
      create_client<std_srvs::srv::Trigger>(depth_hold_enable_roll_pitch_service_name_);
    depth_hold_disable_roll_pitch_client_ =
      create_client<std_srvs::srv::Trigger>(depth_hold_disable_roll_pitch_service_name_);
    alpha_left_forward_velocity_command_pub_ = create_publisher<Float64MultiArrayMsg>(
      alpha_left_forward_velocity_command_topic_,
      rclcpp::SystemDefaultsQoS());
    alpha_right_forward_velocity_command_pub_ = create_publisher<Float64MultiArrayMsg>(
      alpha_right_forward_velocity_command_topic_,
      rclcpp::SystemDefaultsQoS());
    ///////// Create publishers for any additional controllers as needed
    newton_gripper_command_pub_ = create_publisher<Float64MultiArrayMsg>(newton_gripper_command_topic_,rclcpp::SystemDefaultsQoS());
    servo_zip_command_pub_ = create_publisher<Float64MultiArrayMsg>(servo_zip_command_topic_,rclcpp::SystemDefaultsQoS());
    servo_camera_command_pub_ = create_publisher<Float64MultiArrayMsg>(servo_camera_command_topic_,rclcpp::SystemDefaultsQoS());
    //////////
    
    
    
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_),
      std::bind(&CirtesubTeleop::timerCallback, this));

    if (alpha_forward_command_rate_ <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "Invalid Alpha forward command rate %.3f Hz, using 10.0 Hz.",
        alpha_forward_command_rate_);
      alpha_forward_command_rate_ = 10.0;
    }

    alpha_forward_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / alpha_forward_command_rate_),
      std::bind(&CirtesubTeleop::alphaForwardTimerCallback, this));
    
    if (servo_command_rate_ <= 0.0) {
    servo_command_rate_ = 20.0;
    }

    servo_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / servo_command_rate_),
      std::bind(&CirtesubTeleop::servoTimerCallback, this));


    RCLCPP_INFO(
      get_logger(),
      "Teleop ready. R3 selects AUV mode, L3 selects arm mode, RB+X toggles '%s', RB+B toggles '%s', RB+Y toggles '%s', RB+A toggles '%s', RB+LB toggles '%s', RB+hat left selects '%s', RB+hat right selects '%s', active command topic='%s'.",
      body_velocity_controller_name_.c_str(),
      position_hold_controller_name_.c_str(),
      stabilize_controller_name_.c_str(),
      depth_hold_controller_name_.c_str(),
      body_force_controller_name_.c_str(),
      alpha_left_forward_velocity_controller_name_.c_str(),
      alpha_right_forward_velocity_controller_name_.c_str(),
      active_command_topic_.c_str());
  }

private:
  using JoyMsg = sensor_msgs::msg::Joy;
  using TwistMsg = geometry_msgs::msg::Twist;
  using WrenchMsg = geometry_msgs::msg::Wrench;
  using Float64MultiArrayMsg = std_msgs::msg::Float64MultiArray;
  using ListControllersSrv = controller_manager_msgs::srv::ListControllers;
  using SwitchControllerSrv = controller_manager_msgs::srv::SwitchController;

  enum class AlphaForwardControllerSelection
  {
    None,
    Left,
    Right
  };

  enum class TeleopMode
  {
    Auv,
    Arm
  };

  enum class CommandOutputMode
  {
    None,
    Twist,
    Wrench
  };

  void updateTwistPublisher(const std::string & topic_name)
  {
    active_command_topic_ = topic_name;
    wrench_command_pub_.reset();
    twist_command_pub_ = create_publisher<TwistMsg>(
      active_command_topic_,
      rclcpp::SystemDefaultsQoS());
    command_output_mode_ = CommandOutputMode::Twist;
  }

  void updateWrenchPublisher(const std::string & topic_name)
  {
    active_command_topic_ = topic_name;
    twist_command_pub_.reset();
    wrench_command_pub_ = create_publisher<WrenchMsg>(
      active_command_topic_,
      rclcpp::SystemDefaultsQoS());
    command_output_mode_ = CommandOutputMode::Wrench;
  }

  void publishZeroFeedforward()
  {
    if (command_output_mode_ == CommandOutputMode::Twist && twist_command_pub_) {
      twist_command_pub_->publish(TwistMsg{});
    }
    if (command_output_mode_ == CommandOutputMode::Wrench && wrench_command_pub_) {
      wrench_command_pub_->publish(WrenchMsg{});
    }
  }

  void joyCallback(const JoyMsg::SharedPtr msg)
  {
    last_joy_msg_ = msg;

    if (
      !isValidButtonIndex(msg->buttons, a_button_) ||
      !isValidButtonIndex(msg->buttons, b_button_) ||
      !isValidButtonIndex(msg->buttons, x_button_) ||
      !isValidButtonIndex(msg->buttons, lb_button_) ||
      !isValidButtonIndex(msg->buttons, rb_button_) ||
      !isValidButtonIndex(msg->buttons, y_button_) ||
      !isValidButtonIndex(msg->buttons, left_stick_button_) ||
      !isValidButtonIndex(msg->buttons, right_stick_button_))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Configured button indices are out of range for the current joystick message.");
      return;
    }

    const bool a_pressed = msg->buttons[static_cast<size_t>(a_button_)] != 0;
    const bool b_pressed = msg->buttons[static_cast<size_t>(b_button_)] != 0;
    const bool x_pressed = msg->buttons[static_cast<size_t>(x_button_)] != 0;
    const bool lb_pressed = msg->buttons[static_cast<size_t>(lb_button_)] != 0;
    const bool rb_pressed = msg->buttons[static_cast<size_t>(rb_button_)] != 0;
    const bool y_pressed = msg->buttons[static_cast<size_t>(y_button_)] != 0;
    ///
    const bool gripper_close_combo_pressed = lb_pressed && a_pressed;
    const bool gripper_open_combo_pressed = lb_pressed && y_pressed;
    const bool zip_decrease_combo_pressed = lb_pressed && x_pressed;
    const bool zip_increase_combo_pressed = lb_pressed && b_pressed;
    ///
    const bool left_stick_pressed =
      msg->buttons[static_cast<size_t>(left_stick_button_)] != 0;
    const bool right_stick_pressed =
      msg->buttons[static_cast<size_t>(right_stick_button_)] != 0;





    if (left_stick_pressed && !last_left_stick_button_state_) {
      setTeleopMode(TeleopMode::Arm);
    }

    if (right_stick_pressed && !last_right_stick_button_state_) {
      setTeleopMode(TeleopMode::Auv);
    }

    const bool body_velocity_combo_pressed = rb_pressed && x_pressed;
    const bool position_hold_combo_pressed = rb_pressed && b_pressed;
    const bool stabilize_combo_pressed = rb_pressed && y_pressed;
    const bool depth_hold_combo_pressed = rb_pressed && a_pressed;
    const bool body_force_combo_pressed = rb_pressed && lb_pressed;

    if (teleop_mode_ == TeleopMode::Auv &&
      body_velocity_combo_pressed && !last_body_velocity_combo_state_)
    {
      requestBodyVelocityToggle();
    }

    if (teleop_mode_ == TeleopMode::Auv &&
      position_hold_combo_pressed && !last_position_hold_combo_state_)
    {
      requestPositionHoldToggle();
    }

    if (teleop_mode_ == TeleopMode::Auv &&
      stabilize_combo_pressed && !last_stabilize_combo_state_)
    {
      requestStabilizeToggle();
    }

    if (teleop_mode_ == TeleopMode::Auv &&
      depth_hold_combo_pressed && !last_depth_hold_combo_state_)
    {
      requestDepthHoldToggle();
    }

    if (teleop_mode_ == TeleopMode::Auv &&
      body_force_combo_pressed && !last_body_force_combo_state_)
    {
      requestBodyForceToggle();
    }

    if (teleop_mode_ == TeleopMode::Arm) {
      processAlphaForwardControllerSelection(*msg, rb_pressed);
    } else {
      last_hat_horizontal_state_ = 0;
    }

    if (teleop_mode_ == TeleopMode::Auv) {
      processHatCommands(*msg);
    } else {
      last_hat_vertical_state_ = 0;
    }
    ///////
    processServoCommands(
      *msg,
      gripper_close_combo_pressed,
      gripper_open_combo_pressed,
      zip_decrease_combo_pressed,
      zip_increase_combo_pressed);
    ///////


    last_body_velocity_combo_state_ = body_velocity_combo_pressed;
    last_position_hold_combo_state_ = position_hold_combo_pressed;
    last_stabilize_combo_state_ = stabilize_combo_pressed;
    last_depth_hold_combo_state_ = depth_hold_combo_pressed;
    last_body_force_combo_state_ = body_force_combo_pressed;
    last_left_stick_button_state_ = left_stick_pressed;
    last_right_stick_button_state_ = right_stick_pressed;
  }

  void timerCallback()
  {
    if (teleop_mode_ != TeleopMode::Auv) {
      return;
    }

    const bool direct_body_force_enabled = body_force_enabled_ && !body_velocity_enabled_ &&
      !position_hold_enabled_ && !stabilize_enabled_ && !depth_hold_enabled_;

    if (!direct_body_force_enabled && !body_velocity_enabled_ && !position_hold_enabled_ &&
      !stabilize_enabled_ && !depth_hold_enabled_)
    {
      return;
    }

    TwistMsg twist_cmd;
    WrenchMsg wrench_cmd;
    if (last_joy_msg_ != nullptr) {
      const bool lb_pressed = isValidButtonIndex(last_joy_msg_->buttons, lb_button_) &&
        last_joy_msg_->buttons[static_cast<size_t>(lb_button_)] != 0;

      if (lb_pressed) {
        twist_cmd.angular.x = readAxis(last_joy_msg_->axes, roll_axis_) * roll_scale_;
        twist_cmd.angular.y = readAxis(last_joy_msg_->axes, pitch_axis_) * pitch_scale_;
        wrench_cmd.torque.x = twist_cmd.angular.x;
        wrench_cmd.torque.y = twist_cmd.angular.y;
      } else {
        twist_cmd.linear.x = readAxis(last_joy_msg_->axes, surge_axis_) * surge_scale_;
        twist_cmd.linear.y = readAxis(last_joy_msg_->axes, sway_axis_) * sway_scale_;
        twist_cmd.linear.z = -readAxis(last_joy_msg_->axes, heave_axis_) * heave_scale_;
        twist_cmd.angular.z = readAxis(last_joy_msg_->axes, yaw_axis_) * yaw_scale_;
        wrench_cmd.force.x = twist_cmd.linear.x;
        wrench_cmd.force.y = twist_cmd.linear.y;
        wrench_cmd.force.z = twist_cmd.linear.z;
        wrench_cmd.torque.z = twist_cmd.angular.z;
      }
    }

    if (direct_body_force_enabled && active_command_topic_ != body_force_command_topic_) {
      updateWrenchPublisher(body_force_command_topic_);
    }

    if (command_output_mode_ == CommandOutputMode::Twist && twist_command_pub_) {
      twist_command_pub_->publish(twist_cmd);
    } else if (command_output_mode_ == CommandOutputMode::Wrench && wrench_command_pub_) {
      if (direct_body_force_enabled) {
        wrench_cmd.force.x *= body_force_feedforward_gain_x_;
        wrench_cmd.force.y *= body_force_feedforward_gain_y_;
        wrench_cmd.force.z *= body_force_feedforward_gain_z_;
        wrench_cmd.torque.x *= body_force_feedforward_gain_roll_;
        wrench_cmd.torque.y *= body_force_feedforward_gain_pitch_;
        wrench_cmd.torque.z *= body_force_feedforward_gain_yaw_;
        wrench_command_pub_->publish(wrench_cmd);
        return;
      }

      double feedforward_gain_x = stabilize_feedforward_gain_x_;
      double feedforward_gain_y = stabilize_feedforward_gain_y_;
      double feedforward_gain_z = stabilize_feedforward_gain_z_;
      double feedforward_gain_roll = stabilize_feedforward_gain_roll_;
      double feedforward_gain_pitch = stabilize_feedforward_gain_pitch_;
      double feedforward_gain_yaw = stabilize_feedforward_gain_yaw_;

      if (depth_hold_enabled_ && !position_hold_enabled_) {
        feedforward_gain_x = depth_hold_feedforward_gain_x_;
        feedforward_gain_y = depth_hold_feedforward_gain_y_;
        feedforward_gain_z = depth_hold_feedforward_gain_z_;
        feedforward_gain_roll = depth_hold_feedforward_gain_roll_;
        feedforward_gain_pitch = depth_hold_feedforward_gain_pitch_;
        feedforward_gain_yaw = depth_hold_feedforward_gain_yaw_;
      }

      wrench_cmd.force.x *= feedforward_gain_x;
      wrench_cmd.force.y *= feedforward_gain_y;
      wrench_cmd.force.z *= feedforward_gain_z;
      wrench_cmd.torque.x *= feedforward_gain_roll;
      wrench_cmd.torque.y *= feedforward_gain_pitch;
      wrench_cmd.torque.z *= feedforward_gain_yaw;
      wrench_command_pub_->publish(wrench_cmd);
    }
  }

  void alphaForwardTimerCallback()
  {
    if (teleop_mode_ != TeleopMode::Arm) {
      return;
    }

    if (alpha_forward_controller_selection_ == AlphaForwardControllerSelection::None ||
      last_joy_msg_ == nullptr)
    {
      return;
    }

    double axis_b_command = readAxis(last_joy_msg_->axes, yaw_axis_);
    double axis_c_command = readAxis(last_joy_msg_->axes, heave_axis_);
    double axis_d_command = readAxis(last_joy_msg_->axes, sway_axis_);
    double axis_e_command = readAxis(last_joy_msg_->axes, surge_axis_);

    if (alpha_forward_controller_selection_ == AlphaForwardControllerSelection::Left) {
      axis_d_command = -axis_d_command;
    } else if (alpha_forward_controller_selection_ == AlphaForwardControllerSelection::Right) {
      axis_e_command = -axis_e_command;
    }

    Float64MultiArrayMsg command_msg;
    command_msg.data = {
      0.0,
      axis_b_command,
      axis_c_command,
      axis_d_command,
      axis_e_command};

    if (alpha_forward_controller_selection_ == AlphaForwardControllerSelection::Left) {
      alpha_left_forward_velocity_command_pub_->publish(command_msg);
    } else if (alpha_forward_controller_selection_ == AlphaForwardControllerSelection::Right) {
      alpha_right_forward_velocity_command_pub_->publish(command_msg);
    }
  }
  void servoTimerCallback()
  {
    publishServoCommand(newton_gripper_command_pub_, newton_gripper_command_);
    publishServoCommand(servo_zip_command_pub_, servo_zip_command_);
    publishServoCommand(servo_camera_command_pub_, servo_camera_command_);
  }

  void publishServoCommand(
    const rclcpp::Publisher<Float64MultiArrayMsg>::SharedPtr & publisher,
    double command)
  {
    if (!publisher) {
      return;
    }

    Float64MultiArrayMsg msg;
    msg.data = {command};
    publisher->publish(msg);
  }
  //
  void processServoCommands(
  const JoyMsg & msg,
  bool gripper_close_pressed,
  bool gripper_open_pressed,
  bool zip_decrease_pressed,
  bool zip_increase_pressed)
  {
    if (gripper_close_pressed && !last_gripper_close_combo_state_) {
      newton_gripper_command_ = -1.0;
    }

    if (gripper_open_pressed && !last_gripper_open_combo_state_) {
      newton_gripper_command_ = 1.0;
    }

    if (zip_decrease_pressed && !last_zip_decrease_combo_state_) {
      servo_zip_command_ = clampServoCommand(servo_zip_command_ - servo_step_);
    }

    if (zip_increase_pressed && !last_zip_increase_combo_state_) {
      servo_zip_command_ = clampServoCommand(servo_zip_command_ + servo_step_);
    }
    //////
    const bool lb_pressed =
    isValidButtonIndex(msg.buttons, lb_button_) &&
    msg.buttons[static_cast<size_t>(lb_button_)] != 0;

    const double hat_horizontal = readAxis(msg.axes, hat_horizontal_axis_);
    int camera_hat_state = 0;
    if (hat_horizontal > 0.5) {
      camera_hat_state = 1;
    } else if (hat_horizontal < -0.5) {
      camera_hat_state = -1;
    }

    if (lb_pressed && camera_hat_state != last_servo_camera_hat_state_) {
      if (camera_hat_state > 0) {
        servo_camera_command_ = clampServoCommand(servo_camera_command_ + servo_step_);
      } else if (camera_hat_state < 0) {
        servo_camera_command_ = clampServoCommand(servo_camera_command_ - servo_step_);
      }
    }
    //////

    last_gripper_close_combo_state_ = gripper_close_pressed;
    last_gripper_open_combo_state_ = gripper_open_pressed;
    last_zip_decrease_combo_state_ = zip_decrease_pressed;
    last_zip_increase_combo_state_ = zip_increase_pressed;
    last_servo_camera_hat_state_ = camera_hat_state;
  }

  //
  ////////
    double clampServoCommand(double value) const
  {
    return std::clamp(value, -1.0, 1.0);
  }
  ///////

  using ControllerStateMap = std::unordered_map<std::string, std::string>;

  bool isControllerActive(
    const ControllerStateMap & controller_states,
    const std::string & controller_name) const
  {
    const auto iterator = controller_states.find(controller_name);
    return iterator != controller_states.end() && iterator->second == "active";
  }

  void syncAuvControllerStates(const ControllerStateMap & controller_states)
  {
    body_force_enabled_ = isControllerActive(controller_states, body_force_controller_name_);
    body_velocity_enabled_ = isControllerActive(controller_states, body_velocity_controller_name_);
    position_hold_enabled_ = isControllerActive(controller_states, position_hold_controller_name_);
    stabilize_enabled_ = isControllerActive(controller_states, stabilize_controller_name_);
    depth_hold_enabled_ = isControllerActive(controller_states, depth_hold_controller_name_);
  }

  void requestControllerStates(
    std::function<void(const ControllerStateMap &)> response_callback,
    const char * busy_message)
  {
    if (switch_in_progress_) {
      RCLCPP_WARN(get_logger(), "%s", busy_message);
      return;
    }

    if (!list_controllers_client_->wait_for_service(std::chrono::milliseconds(200))) {
      RCLCPP_WARN(
        get_logger(),
        "Controller list service '%s' is not available.",
        controller_list_service_.c_str());
      return;
    }

    switch_in_progress_ = true;
    auto request = std::make_shared<ListControllersSrv::Request>();
    const auto future = list_controllers_client_->async_send_request(
      request,
      [this, response_callback](rclcpp::Client<ListControllersSrv>::SharedFuture future_response)
      {
        switch_in_progress_ = false;

        ControllerStateMap controller_states;
        for (const auto & controller : future_response.get()->controller) {
          controller_states[controller.name] = controller.state;
        }

        syncAuvControllerStates(controller_states);
        response_callback(controller_states);
      });

    (void)future;
  }

  void deduplicateControllers(std::vector<std::string> & controller_names) const
  {
    std::sort(controller_names.begin(), controller_names.end());
    controller_names.erase(
      std::unique(controller_names.begin(), controller_names.end()),
      controller_names.end());
  }

  void requestStabilizeToggle()
  {
    requestControllerStates(
      [this](const ControllerStateMap & controller_states)
      {
        requestStabilizeState(
          !isControllerActive(controller_states, stabilize_controller_name_),
          controller_states);
      },
      "Ignoring toggle request because a controller query is already in progress.");
  }

  void requestStabilizeState(bool enable, const ControllerStateMap & controller_states)
  {
    std::vector<std::string> activate_controllers;
    std::vector<std::string> deactivate_controllers;

    if (enable) {
      if (!isControllerActive(controller_states, body_force_controller_name_)) {
        activate_controllers.push_back(body_force_controller_name_);
      }
      if (isControllerActive(controller_states, body_velocity_controller_name_)) {
        deactivate_controllers.push_back(body_velocity_controller_name_);
      }
      if (isControllerActive(controller_states, position_hold_controller_name_)) {
        deactivate_controllers.push_back(position_hold_controller_name_);
        deactivate_controllers.push_back(body_velocity_controller_name_);
      }
      if (isControllerActive(controller_states, depth_hold_controller_name_)) {
        deactivate_controllers.push_back(depth_hold_controller_name_);
      }
      if (!isControllerActive(controller_states, stabilize_controller_name_)) {
        activate_controllers.push_back(stabilize_controller_name_);
      }
    } else {
      if (isControllerActive(controller_states, body_velocity_controller_name_)) {
        deactivate_controllers.push_back(body_velocity_controller_name_);
      }
      if (isControllerActive(controller_states, stabilize_controller_name_)) {
        deactivate_controllers.push_back(stabilize_controller_name_);
      }
    }

    deduplicateControllers(activate_controllers);
    deduplicateControllers(deactivate_controllers);

    if (activate_controllers.empty() && deactivate_controllers.empty()) {
      syncAuvControllerStates(controller_states);
      RCLCPP_INFO(
        get_logger(),
        "Controller '%s' already matches the requested state.",
        stabilize_controller_name_.c_str());
      return;
    }

    sendSwitchRequest(
      activate_controllers,
      deactivate_controllers,
      [this, enable](rclcpp::Client<SwitchControllerSrv>::SharedFuture future_response)
      {
        const auto response = future_response.get();
        if (!response->ok) {
          RCLCPP_ERROR(
            get_logger(),
            "Failed to %s controller '%s'.",
            enable ? "activate" : "deactivate",
            stabilize_controller_name_.c_str());
          return;
        }

        if (enable) {
          body_force_enabled_ = true;
          body_velocity_enabled_ = false;
          position_hold_enabled_ = false;
          depth_hold_enabled_ = false;
          updateWrenchPublisher(stabilize_feedforward_topic_);
        } else {
          body_velocity_enabled_ = false;
        }
        stabilize_enabled_ = enable;
        RCLCPP_INFO(
          get_logger(),
          "Controller '%s' %s.",
          stabilize_controller_name_.c_str(),
          stabilize_enabled_ ? "activated" : "deactivated");

        if (!stabilize_enabled_) {
          publishZeroFeedforward();
        }
      });
  }

  void requestDepthHoldToggle()
  {
    requestControllerStates(
      [this](const ControllerStateMap & controller_states)
      {
        requestDepthHoldState(
          !isControllerActive(controller_states, depth_hold_controller_name_),
          controller_states);
      },
      "Ignoring toggle request because a controller query is already in progress.");
  }

  void requestDepthHoldState(bool enable, const ControllerStateMap & controller_states)
  {
    std::vector<std::string> activate_controllers;
    std::vector<std::string> deactivate_controllers;

    if (enable) {
      if (!isControllerActive(controller_states, body_force_controller_name_)) {
        activate_controllers.push_back(body_force_controller_name_);
      }
      if (isControllerActive(controller_states, body_velocity_controller_name_)) {
        deactivate_controllers.push_back(body_velocity_controller_name_);
      }
      if (isControllerActive(controller_states, position_hold_controller_name_)) {
        deactivate_controllers.push_back(position_hold_controller_name_);
        deactivate_controllers.push_back(body_velocity_controller_name_);
      }
      if (isControllerActive(controller_states, stabilize_controller_name_)) {
        deactivate_controllers.push_back(stabilize_controller_name_);
      }
      if (!isControllerActive(controller_states, depth_hold_controller_name_)) {
        activate_controllers.push_back(depth_hold_controller_name_);
      }
    } else {
      if (isControllerActive(controller_states, depth_hold_controller_name_)) {
        deactivate_controllers.push_back(depth_hold_controller_name_);
      }
    }

    deduplicateControllers(activate_controllers);
    deduplicateControllers(deactivate_controllers);

    if (activate_controllers.empty() && deactivate_controllers.empty()) {
      syncAuvControllerStates(controller_states);
      RCLCPP_INFO(
        get_logger(),
        "Controller '%s' already matches the requested state.",
        depth_hold_controller_name_.c_str());
      return;
    }

    sendSwitchRequest(
      activate_controllers,
      deactivate_controllers,
      [this, enable](rclcpp::Client<SwitchControllerSrv>::SharedFuture future_response)
      {
        const auto response = future_response.get();
        if (!response->ok) {
          RCLCPP_ERROR(
            get_logger(),
            "Failed to %s controller '%s'.",
            enable ? "activate" : "deactivate",
            depth_hold_controller_name_.c_str());
          return;
        }

        if (enable) {
          body_force_enabled_ = true;
          body_velocity_enabled_ = false;
          position_hold_enabled_ = false;
          stabilize_enabled_ = false;
          updateWrenchPublisher(depth_hold_feedforward_topic_);
        }
        depth_hold_enabled_ = enable;
        RCLCPP_INFO(
          get_logger(),
          "Controller '%s' %s.",
          depth_hold_controller_name_.c_str(),
          depth_hold_enabled_ ? "activated" : "deactivated");

        if (!depth_hold_enabled_) {
          publishZeroFeedforward();
          updateWrenchPublisher(stabilize_feedforward_topic_);
        }
      });
  }

  void requestBodyVelocityToggle()
  {
    requestControllerStates(
      [this](const ControllerStateMap & controller_states)
      {
        requestBodyVelocityState(
          !isControllerActive(controller_states, body_velocity_controller_name_),
          controller_states);
      },
      "Ignoring toggle request because a controller query is already in progress.");
  }

  void requestBodyVelocityState(bool enable, const ControllerStateMap & controller_states)
  {
    std::vector<std::string> activate_controllers;
    std::vector<std::string> deactivate_controllers;

    if (enable) {
      if (!isControllerActive(controller_states, body_force_controller_name_)) {
        activate_controllers.push_back(body_force_controller_name_);
      }
      if (isControllerActive(controller_states, position_hold_controller_name_)) {
        deactivate_controllers.push_back(position_hold_controller_name_);
      }
      if (!isControllerActive(controller_states, position_hold_controller_name_) &&
        !isControllerActive(controller_states, body_velocity_controller_name_))
      {
        activate_controllers.push_back(body_velocity_controller_name_);
      }
      if (isControllerActive(controller_states, stabilize_controller_name_)) {
        deactivate_controllers.push_back(stabilize_controller_name_);
      }
      if (!isControllerActive(controller_states, depth_hold_controller_name_)) {
        activate_controllers.push_back(depth_hold_controller_name_);
      }
    } else {
      if (isControllerActive(controller_states, body_velocity_controller_name_)) {
        deactivate_controllers.push_back(body_velocity_controller_name_);
      }
    }

    deduplicateControllers(activate_controllers);
    deduplicateControllers(deactivate_controllers);

    if (activate_controllers.empty() && deactivate_controllers.empty()) {
      syncAuvControllerStates(controller_states);
      RCLCPP_INFO(
        get_logger(),
        "Controller '%s' already matches the requested state.",
        body_velocity_controller_name_.c_str());
      return;
    }

    sendSwitchRequest(
      activate_controllers,
      deactivate_controllers,
      [this, enable](rclcpp::Client<SwitchControllerSrv>::SharedFuture future_response)
      {
        const auto response = future_response.get();
        if (!response->ok) {
          RCLCPP_ERROR(
            get_logger(),
            "Failed to %s controller '%s'.",
            enable ? "activate" : "deactivate",
            body_velocity_controller_name_.c_str());
          return;
        }

        if (enable) {
          body_force_enabled_ = true;
          stabilize_enabled_ = false;
          position_hold_enabled_ = false;
          depth_hold_enabled_ = true;
          updateTwistPublisher(body_velocity_setpoint_topic_);
        }
        body_velocity_enabled_ = enable;
        RCLCPP_INFO(
          get_logger(),
          "Controller '%s' %s.",
          body_velocity_controller_name_.c_str(),
          body_velocity_enabled_ ? "activated" : "deactivated");

        if (!body_velocity_enabled_) {
          publishZeroFeedforward();
          depth_hold_enabled_ = true;
          updateWrenchPublisher(depth_hold_feedforward_topic_);
        }
      });
  }

  void requestPositionHoldToggle()
  {
    requestControllerStates(
      [this](const ControllerStateMap & controller_states)
      {
        requestPositionHoldState(
          !isControllerActive(controller_states, position_hold_controller_name_),
          controller_states);
      },
      "Ignoring toggle request because a controller query is already in progress.");
  }

  void requestPositionHoldState(bool enable, const ControllerStateMap & controller_states)
  {
    std::vector<std::string> activate_controllers;
    std::vector<std::string> deactivate_controllers;

    if (enable) {
      if (!isControllerActive(controller_states, body_force_controller_name_)) {
        activate_controllers.push_back(body_force_controller_name_);
      }
      if (isControllerActive(controller_states, stabilize_controller_name_)) {
        deactivate_controllers.push_back(stabilize_controller_name_);
      }
      if (!isControllerActive(controller_states, depth_hold_controller_name_)) {
        activate_controllers.push_back(depth_hold_controller_name_);
      }
      if (!isControllerActive(controller_states, body_velocity_controller_name_)) {
        activate_controllers.push_back(body_velocity_controller_name_);
      }
      if (!isControllerActive(controller_states, position_hold_controller_name_)) {
        activate_controllers.push_back(position_hold_controller_name_);
      }
    } else {
      if (isControllerActive(controller_states, position_hold_controller_name_)) {
        deactivate_controllers.push_back(position_hold_controller_name_);
      }
      if (isControllerActive(controller_states, body_velocity_controller_name_)) {
        deactivate_controllers.push_back(body_velocity_controller_name_);
      }
    }

    deduplicateControllers(activate_controllers);
    deduplicateControllers(deactivate_controllers);

    if (activate_controllers.empty() && deactivate_controllers.empty()) {
      syncAuvControllerStates(controller_states);
      RCLCPP_INFO(
        get_logger(),
        "Controller '%s' already matches the requested state.",
        position_hold_controller_name_.c_str());
      return;
    }

    sendSwitchRequest(
      activate_controllers,
      deactivate_controllers,
      [this, enable](rclcpp::Client<SwitchControllerSrv>::SharedFuture future_response)
      {
        const auto response = future_response.get();
        if (!response->ok) {
          RCLCPP_ERROR(
            get_logger(),
            "Failed to %s controller '%s'.",
            enable ? "activate" : "deactivate",
            position_hold_controller_name_.c_str());
          return;
        }

        if (enable) {
          body_force_enabled_ = true;
          body_velocity_enabled_ = false;
          stabilize_enabled_ = false;
          depth_hold_enabled_ = true;
          position_hold_enabled_ = true;
          updateTwistPublisher(position_hold_feedforward_topic_);
        } else {
          position_hold_enabled_ = false;
          publishZeroFeedforward();
          depth_hold_enabled_ = true;
          updateWrenchPublisher(depth_hold_feedforward_topic_);
        }

        RCLCPP_INFO(
          get_logger(),
          "Controller '%s' %s.",
          position_hold_controller_name_.c_str(),
          position_hold_enabled_ ? "activated" : "deactivated");
      });
  }

  void requestBodyForceToggle()
  {
    requestControllerStates(
      [this](const ControllerStateMap & controller_states)
      {
        requestBodyForceState(
          !isControllerActive(controller_states, body_force_controller_name_),
          controller_states);
      },
      "Ignoring toggle request because a controller query is already in progress.");
  }

  void requestBodyForceState(bool enable, const ControllerStateMap & controller_states)
  {
    std::vector<std::string> activate_controllers;
    std::vector<std::string> deactivate_controllers;

    if (enable) {
      if (!isControllerActive(controller_states, body_force_controller_name_)) {
        activate_controllers.push_back(body_force_controller_name_);
      }
    } else {
      if (isControllerActive(controller_states, body_velocity_controller_name_)) {
        deactivate_controllers.push_back(body_velocity_controller_name_);
      }
      if (isControllerActive(controller_states, position_hold_controller_name_)) {
        deactivate_controllers.push_back(position_hold_controller_name_);
        deactivate_controllers.push_back(body_velocity_controller_name_);
      }
      if (isControllerActive(controller_states, stabilize_controller_name_)) {
        deactivate_controllers.push_back(stabilize_controller_name_);
      }
      if (isControllerActive(controller_states, depth_hold_controller_name_)) {
        deactivate_controllers.push_back(depth_hold_controller_name_);
      }
      if (isControllerActive(controller_states, body_force_controller_name_)) {
        deactivate_controllers.push_back(body_force_controller_name_);
      }
    }

    deduplicateControllers(activate_controllers);
    deduplicateControllers(deactivate_controllers);

    if (activate_controllers.empty() && deactivate_controllers.empty()) {
      syncAuvControllerStates(controller_states);
      RCLCPP_INFO(
        get_logger(),
        "Controller '%s' already matches the requested state.",
        body_force_controller_name_.c_str());
      return;
    }

    sendSwitchRequest(
      activate_controllers,
      deactivate_controllers,
      [this, enable](rclcpp::Client<SwitchControllerSrv>::SharedFuture future_response)
      {
        const auto response = future_response.get();
        if (!response->ok) {
          RCLCPP_ERROR(
            get_logger(),
            "Failed to %s controller '%s'.",
            enable ? "activate" : "deactivate",
            body_force_controller_name_.c_str());
          return;
        }

        body_force_enabled_ = enable;
        if (!enable) {
          body_velocity_enabled_ = false;
          position_hold_enabled_ = false;
          stabilize_enabled_ = false;
          depth_hold_enabled_ = false;
          publishZeroFeedforward();
          updateWrenchPublisher(stabilize_feedforward_topic_);
        }

        RCLCPP_INFO(
          get_logger(),
          "Controller '%s' %s.",
          body_force_controller_name_.c_str(),
          body_force_enabled_ ? "activated" : "deactivated");
      });
  }

  void sendSwitchRequest(
    const std::vector<std::string> & activate_controllers,
    const std::vector<std::string> & deactivate_controllers,
    std::function<void(rclcpp::Client<SwitchControllerSrv>::SharedFuture)> response_callback)
  {
    if (switch_in_progress_) {
      RCLCPP_WARN(get_logger(), "Ignoring toggle request because a switch is already in progress.");
      return;
    }

    if (!switch_controller_client_->wait_for_service(std::chrono::milliseconds(200))) {
      RCLCPP_WARN(
        get_logger(),
        "Controller switch service '%s' is not available.",
        controller_switch_service_.c_str());
      return;
    }

    auto request = std::make_shared<SwitchControllerSrv::Request>();
    request->activate_controllers = activate_controllers;
    request->deactivate_controllers = deactivate_controllers;
    request->strictness = SwitchControllerSrv::Request::STRICT;
    request->activate_asap = true;
    request->timeout.sec = 2;
    request->timeout.nanosec = 0;

    switch_in_progress_ = true;
    const auto future = switch_controller_client_->async_send_request(
      request,
      [this, response_callback](rclcpp::Client<SwitchControllerSrv>::SharedFuture future_response)
      {
        switch_in_progress_ = false;
        response_callback(future_response);
      });

    (void)future;
  }

  void processAlphaForwardControllerSelection(const JoyMsg & msg, bool rb_pressed)
  {
    if (!rb_pressed) {
      last_hat_horizontal_state_ = 0;
      return;
    }

    const double hat_value = readAxis(msg.axes, hat_horizontal_axis_);
    int current_hat_horizontal_state = 0;
    if (hat_value > 0.5) {
      current_hat_horizontal_state = 1;
    } else if (hat_value < -0.5) {
      current_hat_horizontal_state = -1;
    }

    if (current_hat_horizontal_state != last_hat_horizontal_state_) {
      if (current_hat_horizontal_state > 0) {
        requestAlphaForwardControllerSelection(AlphaForwardControllerSelection::Left);
      } else if (current_hat_horizontal_state < 0) {
        requestAlphaForwardControllerSelection(AlphaForwardControllerSelection::Right);
      }
    }

    last_hat_horizontal_state_ = current_hat_horizontal_state;
  }

  void requestAlphaForwardControllerSelection(AlphaForwardControllerSelection selection)
  {
    if (switch_in_progress_) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring Alpha controller request because a switch is already in progress.");
      return;
    }

    if (selection == alpha_forward_controller_selection_) {
      return;
    }

    if (!list_controllers_client_->wait_for_service(std::chrono::milliseconds(200))) {
      RCLCPP_WARN(
        get_logger(),
        "Controller list service '%s' is not available.",
        controller_list_service_.c_str());
      return;
    }

    switch_in_progress_ = true;
    auto request = std::make_shared<ListControllersSrv::Request>();
    const auto future = list_controllers_client_->async_send_request(
      request,
      [this, selection](rclcpp::Client<ListControllersSrv>::SharedFuture future_response)
      {
        switch_in_progress_ = false;

        std::unordered_map<std::string, std::string> controller_states;
        for (const auto & controller : future_response.get()->controller) {
          controller_states[controller.name] = controller.state;
        }

        const auto is_active = [&controller_states](const std::string & controller_name) {
            const auto iterator = controller_states.find(controller_name);
            return iterator != controller_states.end() && iterator->second == "active";
          };

        std::vector<std::string> activate_controllers;
        std::vector<std::string> deactivate_controllers;

        if (selection == AlphaForwardControllerSelection::Left) {
          if (!is_active(alpha_left_forward_velocity_controller_name_)) {
            activate_controllers.push_back(alpha_left_forward_velocity_controller_name_);
          }
          if (is_active(alpha_right_forward_velocity_controller_name_)) {
            deactivate_controllers.push_back(alpha_right_forward_velocity_controller_name_);
          }
        } else if (selection == AlphaForwardControllerSelection::Right) {
          if (!is_active(alpha_right_forward_velocity_controller_name_)) {
            activate_controllers.push_back(alpha_right_forward_velocity_controller_name_);
          }
          if (is_active(alpha_left_forward_velocity_controller_name_)) {
            deactivate_controllers.push_back(alpha_left_forward_velocity_controller_name_);
          }
        } else {
          if (is_active(alpha_left_forward_velocity_controller_name_)) {
            deactivate_controllers.push_back(alpha_left_forward_velocity_controller_name_);
          }
          if (is_active(alpha_right_forward_velocity_controller_name_)) {
            deactivate_controllers.push_back(alpha_right_forward_velocity_controller_name_);
          }
        }

        if (activate_controllers.empty() && deactivate_controllers.empty()) {
          alpha_forward_controller_selection_ = selection;
          RCLCPP_INFO(
            get_logger(),
            "Active Alpha forward velocity controller: %s.",
            getAlphaForwardControllerLabel(selection).c_str());
          return;
        }

        sendSwitchRequest(
          activate_controllers,
          deactivate_controllers,
          [this, selection](rclcpp::Client<SwitchControllerSrv>::SharedFuture switch_future)
          {
            const auto response = switch_future.get();
            if (!response->ok) {
              RCLCPP_ERROR(get_logger(), "Failed to switch Alpha forward velocity controllers.");
              return;
            }

            alpha_forward_controller_selection_ = selection;
            RCLCPP_INFO(
              get_logger(),
              "Active Alpha forward velocity controller: %s.",
              getAlphaForwardControllerLabel(selection).c_str());
          });
      });

    (void)future;
  }

  void setTeleopMode(TeleopMode mode)
  {
    if (teleop_mode_ == mode) {
      return;
    }

    teleop_mode_ = mode;
    RCLCPP_INFO(
      get_logger(),
      "Teleop mode changed to %s.",
      teleop_mode_ == TeleopMode::Arm ? "arm" : "auv");
  }

  std::string getAlphaForwardControllerLabel(AlphaForwardControllerSelection selection) const
  {
    if (selection == AlphaForwardControllerSelection::Left) {
      return alpha_left_forward_velocity_controller_name_;
    }
    if (selection == AlphaForwardControllerSelection::Right) {
      return alpha_right_forward_velocity_controller_name_;
    }
    return "none";
  }

  void processHatCommands(const JoyMsg & msg)
  {
    if (!stabilize_enabled_ && !depth_hold_enabled_) {
      last_hat_vertical_state_ = 0;
      return;
    }

    const double hat_value = readAxis(msg.axes, hat_vertical_axis_);
    int current_hat_vertical_state = 0;
    if (hat_value > 0.5) {
      current_hat_vertical_state = 1;
    } else if (hat_value < -0.5) {
      current_hat_vertical_state = -1;
    }

    if (current_hat_vertical_state != last_hat_vertical_state_) {
      if (current_hat_vertical_state > 0) {
        requestRollPitchService(getActiveEnableRollPitchClient(), "enable");
      } else if (current_hat_vertical_state < 0) {
        requestRollPitchService(getActiveDisableRollPitchClient(), "disable");
      }
    }

    last_hat_vertical_state_ = current_hat_vertical_state;
  }

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr getActiveEnableRollPitchClient() const
  {
    if (depth_hold_enabled_) {
      return depth_hold_enable_roll_pitch_client_;
    }
    return stabilize_enable_roll_pitch_client_;
  }

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr getActiveDisableRollPitchClient() const
  {
    if (depth_hold_enabled_) {
      return depth_hold_disable_roll_pitch_client_;
    }
    return stabilize_disable_roll_pitch_client_;
  }

  void requestRollPitchService(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    const std::string & action_name)
  {
    if (!client->wait_for_service(std::chrono::milliseconds(200))) {
      RCLCPP_WARN(
        get_logger(),
        "Roll/pitch %s service is not available.",
        action_name.c_str());
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    const auto future = client->async_send_request(
      request,
      [this, action_name](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future_response)
      {
        const auto response = future_response.get();
        if (!response->success) {
          RCLCPP_WARN(
            get_logger(),
            "Failed to %s roll/pitch: %s",
            action_name.c_str(),
            response->message.c_str());
          return;
        }

        RCLCPP_INFO(
          get_logger(),
          "Roll/pitch %s request accepted.",
          action_name.c_str());
      });

    (void)future;
  }

  double readAxis(const std::vector<float> & axes, int index) const
  {
    if (!isValidAxisIndex(axes, index)) {
      return 0.0;
    }

    const double value = static_cast<double>(axes[static_cast<size_t>(index)]);
    return std::fabs(value) < deadzone_ ? 0.0 : value;
  }

  bool isValidAxisIndex(const std::vector<float> & axes, int index) const
  {
    return index >= 0 && static_cast<size_t>(index) < axes.size();
  }

  bool isValidButtonIndex(const std::vector<int32_t> & buttons, int index) const
  {
    return index >= 0 && static_cast<size_t>(index) < buttons.size();
  }

  double rate_{20.0};
  double surge_scale_{1.0};
  double sway_scale_{1.0};
  double heave_scale_{1.0};
  double yaw_scale_{1.0};
  double roll_scale_{1.0};
  double pitch_scale_{1.0};
  double deadzone_{0.05};

  int lb_button_{4};
  int rb_button_{5};
  int a_button_{0};
  int b_button_{1};
  int x_button_{2};
  int y_button_{3};
  int left_stick_button_{9};
  int right_stick_button_{10};
  int hat_horizontal_axis_{6};
  int hat_vertical_axis_{7};
  int surge_axis_{4};
  int sway_axis_{3};
  int yaw_axis_{0};
  int heave_axis_{1};
  int roll_axis_{3};
  int pitch_axis_{4};

  bool body_force_enabled_{false};
  bool body_velocity_enabled_{false};
  bool position_hold_enabled_{false};
  bool stabilize_enabled_{false};
  bool depth_hold_enabled_{false};
  bool last_body_velocity_combo_state_{false};
  bool last_position_hold_combo_state_{false};
  bool last_stabilize_combo_state_{false};
  bool last_depth_hold_combo_state_{false};
  bool last_body_force_combo_state_{false};
  bool last_left_stick_button_state_{false};
  bool last_right_stick_button_state_{false};
  bool switch_in_progress_{false};
  int last_hat_horizontal_state_{0};
  int last_hat_vertical_state_{0};

  std::string joy_topic_;
  std::string controller_switch_service_;
  std::string controller_list_service_;
  std::string body_force_controller_name_;
  std::string body_velocity_controller_name_;
  std::string position_hold_controller_name_;
  std::string stabilize_controller_name_;
  std::string depth_hold_controller_name_;
  std::string alpha_left_forward_velocity_controller_name_;
  std::string alpha_right_forward_velocity_controller_name_;
  std::string active_command_topic_;
  std::string body_force_command_topic_;
  std::string body_velocity_setpoint_topic_;
  std::string position_hold_feedforward_topic_;
  std::string stabilize_feedforward_topic_;
  std::string depth_hold_feedforward_topic_;
  std::string alpha_left_forward_velocity_command_topic_;
  std::string alpha_right_forward_velocity_command_topic_;
  std::string stabilize_enable_roll_pitch_service_name_;
  std::string stabilize_disable_roll_pitch_service_name_;
  std::string depth_hold_enable_roll_pitch_service_name_;
  std::string depth_hold_disable_roll_pitch_service_name_;
  //
  std::string newton_gripper_command_topic_;
  std::string servo_zip_command_topic_;
  std::string servo_camera_command_topic_;

  //


  double alpha_forward_command_rate_{10.0};
  double body_force_feedforward_gain_x_{20.0};
  double body_force_feedforward_gain_y_{20.0};
  double body_force_feedforward_gain_z_{90.0};
  double body_force_feedforward_gain_roll_{20.0};
  double body_force_feedforward_gain_pitch_{20.0};
  double body_force_feedforward_gain_yaw_{1.0};
  double stabilize_feedforward_gain_x_{20.0};
  double stabilize_feedforward_gain_y_{20.0};
  double stabilize_feedforward_gain_z_{90.0};
  double stabilize_feedforward_gain_roll_{20.0};
  double stabilize_feedforward_gain_pitch_{20.0};
  double stabilize_feedforward_gain_yaw_{1.0};
  double depth_hold_feedforward_gain_x_{20.0};
  double depth_hold_feedforward_gain_y_{20.0};
  double depth_hold_feedforward_gain_z_{90.0};
  double depth_hold_feedforward_gain_roll_{20.0};
  double depth_hold_feedforward_gain_pitch_{20.0};
  double depth_hold_feedforward_gain_yaw_{1.0};

  //
  double servo_command_rate_{20.0};
  double servo_step_{0.1};
  double newton_gripper_command_{0.0};
  double servo_zip_command_{0.0};
  double servo_camera_command_{0.0};



  bool last_gripper_close_combo_state_{false};
  bool last_gripper_open_combo_state_{false};
  bool last_zip_decrease_combo_state_{false};
  bool last_zip_increase_combo_state_{false};
  int last_servo_camera_hat_state_{0};
  //

  JoyMsg::SharedPtr last_joy_msg_;
  TeleopMode teleop_mode_{TeleopMode::Auv};
  CommandOutputMode command_output_mode_{CommandOutputMode::None};
  AlphaForwardControllerSelection alpha_forward_controller_selection_{
    AlphaForwardControllerSelection::None};

  rclcpp::Subscription<JoyMsg>::SharedPtr joy_sub_;
  rclcpp::Publisher<TwistMsg>::SharedPtr twist_command_pub_;
  rclcpp::Publisher<WrenchMsg>::SharedPtr wrench_command_pub_;
  rclcpp::Publisher<Float64MultiArrayMsg>::SharedPtr alpha_left_forward_velocity_command_pub_;
  rclcpp::Publisher<Float64MultiArrayMsg>::SharedPtr alpha_right_forward_velocity_command_pub_;
  rclcpp::Client<ListControllersSrv>::SharedPtr list_controllers_client_;
  rclcpp::Client<SwitchControllerSrv>::SharedPtr switch_controller_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stabilize_enable_roll_pitch_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stabilize_disable_roll_pitch_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr depth_hold_enable_roll_pitch_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr depth_hold_disable_roll_pitch_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr alpha_forward_timer_;

  //
  rclcpp::Publisher<Float64MultiArrayMsg>::SharedPtr newton_gripper_command_pub_;
  rclcpp::Publisher<Float64MultiArrayMsg>::SharedPtr servo_zip_command_pub_;
  rclcpp::Publisher<Float64MultiArrayMsg>::SharedPtr servo_camera_command_pub_;
  rclcpp::TimerBase::SharedPtr servo_timer_;

  //
  
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CirtesubTeleop>());
  rclcpp::shutdown();
  return 0;
}
