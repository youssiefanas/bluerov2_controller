#include "bluerov2_controller/rov_controller_node.hpp"

#include <chrono>
#include <functional>
#include <thread>

namespace bluerov2_controller {

RovControllerNode::RovControllerNode(const rclcpp::NodeOptions &options)
    : Node("rov_controller", options) {
  // Declare and load parameters
  double control_rate = this->declare_parameter("control_rate", 20.0);
  int mavros_stream_rate = this->declare_parameter("mavros_stream_rate", 25);

  pwm_neutral_ = this->declare_parameter("pwm_neutral", 1500);
  pwm_min_ = this->declare_parameter("pwm_min", 1100);
  pwm_max_ = this->declare_parameter("pwm_max", 1900);
  pwm_scale_ = this->declare_parameter("pwm_scale", 400);

tilt_angle_min_     = this->declare_parameter("tilt_angle_min", -90.0);
  tilt_angle_max_     = this->declare_parameter("tilt_angle_max",  30.0);
  tilt_angle_initial_ = this->declare_parameter("tilt_angle_initial", 0.0);
  tilt_angle_step_    = this->declare_parameter("tilt_angle_step", 10.0);
  tilt_angle_         = tilt_angle_initial_;

  btn_arm_ = this->declare_parameter("btn_arm", 7);
  btn_disarm_ = this->declare_parameter("btn_disarm", 6);
  btn_manual_mode_ = this->declare_parameter("btn_manual_mode", 3);
  btn_stabilize_mode_ = this->declare_parameter("btn_stabilize_mode", 2);
  btn_depth_hold_mode_ = this->declare_parameter("btn_depth_hold_mode", 0);
  btn_poshold_mode_ = this->declare_parameter("btn_poshold_mode", 1);
  btn_cam_tilt_up_ = this->declare_parameter("btn_cam_tilt_up", 4);
  btn_cam_tilt_down_ = this->declare_parameter("btn_cam_tilt_down", 5);
  btn_cam_tilt_reset_ = this->declare_parameter("btn_cam_tilt_reset", 9);

  service_timeout_sec_ = this->declare_parameter("service_timeout_sec", 4.0);
  aux_timeout_sec_ = this->declare_parameter("aux_timeout_sec", 0.5);
  bool run_init_test = this->declare_parameter("run_initialization_test", false);

  // MAVROS CommandLong service for arm/disarm (cmd 400) and servo control (cmd 183)
  cmd_client_ =
      this->create_client<mavros_msgs::srv::CommandLong>("cmd/command");
  // MAVROS StreamRate service to configure telemetry data rate from the flight controller
  stream_rate_client_ =
      this->create_client<mavros_msgs::srv::StreamRate>("set_stream_rate");
  // MAVROS SetMode service to switch ArduSub flight modes on the Pixhawk
  set_mode_client_ =
      this->create_client<mavros_msgs::srv::SetMode>("set_mode");

  // RC override to send PWM commands to MAVROS for all 8 channels (6 DOF + 2 camera servos)
  rc_override_pub_ =
      this->create_publisher<mavros_msgs::msg::OverrideRCIn>("rc/override", 10);
  // Euler angles (roll, pitch, yaw) in degrees relative to orientation at startup
  angle_degree_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("angle_degree", 10);
  // Depth value from the barometric pressure sensor
  depth_pub_ = this->create_publisher<std_msgs::msg::Float64>("depth", 10);
  // Absolute static pressure (Pa) from the Bar30 sensor
  pressure_pub_ = this->create_publisher<std_msgs::msg::Float64>("pressure", 10);
  // Angular velocity (p, q, r) in rad/s from the IMU gyroscope
  angular_velocity_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("angular_velocity", 10);

  // Subscribers with BEST_EFFORT QoS to match MAVROS and joy_node
  auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                        .reliability(rclcpp::ReliabilityPolicy::BestEffort);

  // Joystick raw input for button events (arm, disarm, mode switch, servo control)
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", sensor_qos,
      std::bind(&RovControllerNode::joy_callback, this, std::placeholders::_1));

  // Velocity commands from teleop_twist_joy mapping joystick axes to surge/sway/heave/roll/pitch/yaw
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", sensor_qos,
      std::bind(&RovControllerNode::vel_callback, this, std::placeholders::_1));

  // Auxiliary cmd_vel (e.g. from plain_pid_controller) summed into vel_callback.
  cmd_vel_aux_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel_aux", sensor_qos,
      std::bind(&RovControllerNode::vel_aux_callback, this,
                std::placeholders::_1));

  // IMU orientation and angular velocity from MAVROS (Pixhawk flight controller)
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", sensor_qos,
      std::bind(&RovControllerNode::imu_callback, this, std::placeholders::_1));

  // Barometer-based relative altitude from MAVROS for depth measurement
  rel_alt_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "global_position/rel_alt", sensor_qos,
      std::bind(&RovControllerNode::rel_alt_callback, this,
                std::placeholders::_1));

  // Absolute static pressure (Pa) from the Bar30 sensor via MAVROS
  pressure_sub_ = this->create_subscription<sensor_msgs::msg::FluidPressure>(
      "imu/static_pressure", sensor_qos,
      std::bind(&RovControllerNode::pressure_callback, this,
                std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "ROV Controller node started");

  // Disarm on startup and set stream rate
  arm_disarm(false);
  set_stream_rate(mavros_stream_rate);

  // Set initial flight mode to MANUAL
  set_flight_mode(FlightMode::MANUAL);

  // Put the camera mount in MAVLink targeting mode so it accepts pitch commands
  set_mount_mode();

  // Run initialization test if requested
  if (run_init_test) {
    run_initialization_test();
  }
}

// ---------------------------------------------------------------------------
// Flight mode helpers
// ---------------------------------------------------------------------------
std::string RovControllerNode::flight_mode_to_string(FlightMode mode) {
  switch (mode) {
  case FlightMode::MANUAL:     return "19";
  case FlightMode::STABILIZE:  return "0";
  case FlightMode::DEPTH_HOLD: return "2";
  case FlightMode::POSHOLD:    return "16";
  }
  return "19";
}

const char * RovControllerNode::flight_mode_name(FlightMode mode) {
  switch (mode) {
  case FlightMode::MANUAL:     return "MANUAL";
  case FlightMode::STABILIZE:  return "STABILIZE";
  case FlightMode::DEPTH_HOLD: return "DEPTH_HOLD";
  case FlightMode::POSHOLD:    return "POSHOLD";
  }
  return "UNKNOWN";
}

void RovControllerNode::set_flight_mode(FlightMode mode) {
  if (!set_mode_client_->wait_for_service(
          std::chrono::duration<double>(service_timeout_sec_))) {
    RCLCPP_WARN(this->get_logger(), "set_mode service not available");
    return;
  }

  auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  request->base_mode = 0;
  request->custom_mode = flight_mode_to_string(mode);

  const char * mode_name = flight_mode_name(mode);
  auto future = set_mode_client_->async_send_request(
      request,
      [this, mode_name](
          rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture result) {
        auto response = result.get();
        if (response->mode_sent) {
          RCLCPP_INFO(this->get_logger(), "Flight mode set to %s", mode_name);
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to set flight mode to %s",
                      mode_name);
        }
      });
}

// ---------------------------------------------------------------------------
// Joystick callback - handles buttons for mode switching, arm/disarm, servos
// ---------------------------------------------------------------------------
void RovControllerNode::joy_callback(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
  auto btn = [&](int index) -> bool {
    return index >= 0 && index < static_cast<int>(msg->buttons.size()) &&
           msg->buttons[index] == 1;
  };

  // Arm / Disarm
  if (btn(btn_disarm_) && armed_) {
    armed_ = false;
    arm_disarm(false);
  }
  if (btn(btn_arm_) && !armed_) {
    armed_ = true;
    arm_disarm(true);
  }

  // Flight mode switching (sends set_mode to the Pixhawk FC via MAVROS)
  if (btn(btn_manual_mode_) && flight_mode_ != FlightMode::MANUAL) {
    flight_mode_ = FlightMode::MANUAL;
    set_flight_mode(FlightMode::MANUAL);
  }
  if (btn(btn_stabilize_mode_) && flight_mode_ != FlightMode::STABILIZE) {
    flight_mode_ = FlightMode::STABILIZE;
    set_flight_mode(FlightMode::STABILIZE);
  }
  if (btn(btn_depth_hold_mode_) && flight_mode_ != FlightMode::DEPTH_HOLD) {
    flight_mode_ = FlightMode::DEPTH_HOLD;
    set_flight_mode(FlightMode::DEPTH_HOLD);
  }
  if (btn(btn_poshold_mode_) && flight_mode_ != FlightMode::POSHOLD) {
    flight_mode_ = FlightMode::POSHOLD;
    set_flight_mode(FlightMode::POSHOLD);
  }

// Camera tilt control (mount pitch in degrees) — edge-triggered so one
  // button press = one step, regardless of joy publish rate.
  bool tilt_up    = btn(btn_cam_tilt_up_);
  bool tilt_dn    = btn(btn_cam_tilt_down_);
  bool tilt_reset = btn(btn_cam_tilt_reset_);

  if (tilt_up && !prev_btn_tilt_up_ && !tilt_dn &&
      tilt_angle_ < tilt_angle_max_) {
    tilt_angle_ = std::min(tilt_angle_ + tilt_angle_step_, tilt_angle_max_);
    send_mount_pitch(tilt_angle_);
    RCLCPP_INFO(this->get_logger(), "Tilt angle: %.1f deg", tilt_angle_);
  }
  if (tilt_dn && !prev_btn_tilt_down_ && tilt_angle_ > tilt_angle_min_) {
    tilt_angle_ = std::max(tilt_angle_ - tilt_angle_step_, tilt_angle_min_);
    send_mount_pitch(tilt_angle_);
    RCLCPP_INFO(this->get_logger(), "Tilt angle: %.1f deg", tilt_angle_);
  }
  if (tilt_reset && !prev_btn_tilt_reset_) {
    tilt_angle_ = tilt_angle_initial_;
    send_mount_pitch(tilt_angle_);
    RCLCPP_INFO(this->get_logger(), "Camera tilt reset");
  }

  prev_btn_tilt_up_    = tilt_up;
  prev_btn_tilt_down_  = tilt_dn;
  prev_btn_tilt_reset_ = tilt_reset;
}

// ---------------------------------------------------------------------------
// Velocity callback - maps joystick velocity (+ optional aux) to PWM
// ---------------------------------------------------------------------------
void RovControllerNode::vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  // Start from the joystick command...
  double lin_x = msg->linear.x;
  double lin_y = msg->linear.y;
  double lin_z = msg->linear.z;
  double ang_x = msg->angular.x;
  double ang_y = msg->angular.y;
  double ang_z = msg->angular.z;

  // ...and add the auxiliary command (e.g. PID hold) if it's fresh enough.
  if (cmd_vel_aux_seen_) {
    const double age = (this->now() - cmd_vel_aux_stamp_).seconds();
    if (aux_timeout_sec_ <= 0.0 || age <= aux_timeout_sec_) {
      lin_x += cmd_vel_aux_.linear.x;
      lin_y += cmd_vel_aux_.linear.y;
      lin_z += cmd_vel_aux_.linear.z;
      ang_x += cmd_vel_aux_.angular.x;
      ang_y += cmd_vel_aux_.angular.y;
      ang_z += cmd_vel_aux_.angular.z;
    }
  }

  int roll  = map_value_scale_saturate(ang_x);
  int pitch = map_value_scale_saturate(ang_y);
  int yaw   = map_value_scale_saturate(-ang_z);
  int surge = map_value_scale_saturate(lin_x);
  int sway  = map_value_scale_saturate(-lin_y);
  int heave = map_value_scale_saturate(lin_z);

  set_override_rcin(pitch, roll, heave, yaw, surge, sway);
}

// ---------------------------------------------------------------------------
// Auxiliary velocity callback - latches latest aux Twist + stamp
// ---------------------------------------------------------------------------
void RovControllerNode::vel_aux_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  cmd_vel_aux_ = *msg;
  cmd_vel_aux_stamp_ = this->now();
  cmd_vel_aux_seen_ = true;
}

// ---------------------------------------------------------------------------
// IMU callback - quaternion to euler, publish angles and angular velocity
// ---------------------------------------------------------------------------
void RovControllerNode::imu_callback(
    const sensor_msgs::msg::Imu::SharedPtr msg) {
  auto [roll, pitch, yaw] =
      quaternion_to_euler(msg->orientation.x, msg->orientation.y,
                          msg->orientation.z, msg->orientation.w);

  // Publish absolute angles in degrees
  geometry_msgs::msg::Twist angle_msg;
  angle_msg.angular.x = roll * 180.0 / M_PI;
  angle_msg.angular.y = pitch * 180.0 / M_PI;
  angle_msg.angular.z = yaw * 180.0 / M_PI;
  angle_degree_pub_->publish(angle_msg);

  // Publish angular velocity
  geometry_msgs::msg::Twist vel_msg;
  vel_msg.angular.x = msg->angular_velocity.x;
  vel_msg.angular.y = msg->angular_velocity.y;
  vel_msg.angular.z = msg->angular_velocity.z;
  angular_velocity_pub_->publish(vel_msg);
}

// ---------------------------------------------------------------------------
// Depth callback
// ---------------------------------------------------------------------------
void RovControllerNode::rel_alt_callback(
    const std_msgs::msg::Float64::SharedPtr msg) {
  std_msgs::msg::Float64 depth_msg;
  depth_msg.data = msg->data;
  depth_pub_->publish(depth_msg);
}

// ---------------------------------------------------------------------------
// Pressure callback
// ---------------------------------------------------------------------------
void RovControllerNode::pressure_callback(
    const sensor_msgs::msg::FluidPressure::SharedPtr msg) {
  std_msgs::msg::Float64 pressure_msg;
  pressure_msg.data = msg->fluid_pressure;
  pressure_pub_->publish(pressure_msg);
}

// ---------------------------------------------------------------------------
// Arm / Disarm via MAV_CMD_COMPONENT_ARM_DISARM (command 400)
// ---------------------------------------------------------------------------
void RovControllerNode::arm_disarm(bool arm) {
  if (!cmd_client_->wait_for_service(
          std::chrono::duration<double>(service_timeout_sec_))) {
    RCLCPP_WARN(this->get_logger(), "cmd/command service not available");
    return;
  }

  auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
  request->command = 400; // MAV_CMD_COMPONENT_ARM_DISARM
  request->param1 = arm ? 1.0f : 0.0f;

  auto future = cmd_client_->async_send_request(
      request,
      [this, arm](
          rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture result) {
        auto response = result.get();
        if (response->success) {
          RCLCPP_INFO(this->get_logger(), "%s succeeded",
                      arm ? "Arm" : "Disarm");
        } else {
          RCLCPP_WARN(this->get_logger(), "%s failed (result: %d)",
                      arm ? "Arm" : "Disarm", response->result);
        }
      });
}

// ---------------------------------------------------------------------------
// Set MAVROS stream rate
// ---------------------------------------------------------------------------
void RovControllerNode::set_stream_rate(int rate) {
  if (!stream_rate_client_->wait_for_service(
          std::chrono::duration<double>(service_timeout_sec_))) {
    RCLCPP_WARN(this->get_logger(), "set_stream_rate service not available");
    return;
  }

  auto request = std::make_shared<mavros_msgs::srv::StreamRate::Request>();
  request->stream_id = 0;
  request->message_rate = rate;
  request->on_off = true;

  auto future = stream_rate_client_->async_send_request(
      request,
      [this, rate](rclcpp::Client<mavros_msgs::srv::StreamRate>::SharedFuture) {
        RCLCPP_INFO(this->get_logger(), "Stream rate set to %d Hz", rate);
      });
}

// ---------------------------------------------------------------------------
// Configure mount for MAVLink targeting (MAV_CMD_DO_MOUNT_CONFIGURE = 204)
// ---------------------------------------------------------------------------
void RovControllerNode::set_mount_mode() {
  if (!cmd_client_->wait_for_service(
          std::chrono::duration<double>(service_timeout_sec_))) {
    RCLCPP_WARN(this->get_logger(),
                "cmd/command service not available for mount configure");
    return;
  }

  auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
  request->command = 204;  // MAV_CMD_DO_MOUNT_CONFIGURE
  request->param1  = 2.0f; // MAV_MOUNT_MODE_MAVLINK_TARGETING
  cmd_client_->async_send_request(request);
  RCLCPP_INFO(this->get_logger(), "Mount set to MAVLink targeting mode");
}

// ---------------------------------------------------------------------------
// Send camera mount pitch (MAV_CMD_DO_MOUNT_CONTROL = 205)
// ---------------------------------------------------------------------------
void RovControllerNode::send_mount_pitch(double pitch_deg) {
  if (!cmd_client_->service_is_ready()) {
    RCLCPP_WARN(this->get_logger(),
                "cmd/command service not ready, skipping mount command");
    return;
  }

  auto request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
  request->command = 205;  // MAV_CMD_DO_MOUNT_CONTROL
  request->param1  = static_cast<float>(pitch_deg); // pitch
  request->param2  = 0.0f;                          // roll
  request->param3  = 0.0f;                          // yaw
  request->param7  = 2.0f; // MAV_MOUNT_MODE_MAVLINK_TARGETING
  cmd_client_->async_send_request(request);
}

// ---------------------------------------------------------------------------
// Set RC channel override
// ---------------------------------------------------------------------------
void RovControllerNode::set_override_rcin(int pitch, int roll, int heave,
                                          int yaw, int surge, int sway) {
  mavros_msgs::msg::OverrideRCIn msg;
  msg.channels[0] = static_cast<uint16_t>(pitch);   // Ch0: pitch
  msg.channels[1] = static_cast<uint16_t>(roll);    // Ch1: roll
  msg.channels[2] = static_cast<uint16_t>(heave);   // Ch2: heave (vertical)
  msg.channels[3] = static_cast<uint16_t>(yaw);     // Ch3: yaw
  msg.channels[4] = static_cast<uint16_t>(surge);   // Ch4: surge (forward/backward)
  msg.channels[5] = static_cast<uint16_t>(sway);    // Ch5: sway (lateral)
  msg.channels[6] = 1500;                           // Ch6: camera pan (neutral)
  msg.channels[7] = 1500;                           // Ch7: camera tilt (neutral)
  rc_override_pub_->publish(msg);
}

// ---------------------------------------------------------------------------
// Initialization test: flash light and sweep camera servo
// ---------------------------------------------------------------------------
void RovControllerNode::run_initialization_test() {
  RCLCPP_INFO(this->get_logger(), "Running initialization test...");

  auto sleep_ms = [](int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
  };

  // Sweep camera mount pitch: initial -> max -> min -> initial
  send_mount_pitch(tilt_angle_initial_);
  sleep_ms(500);
  send_mount_pitch(tilt_angle_max_);
  sleep_ms(500);
  send_mount_pitch(tilt_angle_min_);
  sleep_ms(500);
  send_mount_pitch(tilt_angle_initial_);

  RCLCPP_INFO(this->get_logger(), "Initialization test completed");
}

// ---------------------------------------------------------------------------
// Map velocity [-1, 1] to PWM [pwm_min_, pwm_max_]
// ---------------------------------------------------------------------------
int RovControllerNode::map_value_scale_saturate(double value) const {
  int pwm = static_cast<int>(value * pwm_scale_ + pwm_neutral_);
  return std::clamp(pwm, pwm_min_, pwm_max_);
}

// ---------------------------------------------------------------------------
// Quaternion to Euler (roll, pitch, yaw)
// ---------------------------------------------------------------------------
std::array<double, 3>
RovControllerNode::quaternion_to_euler(double x, double y, double z, double w) {
  double sinr_cosp = 2.0 * (w * x + y * z);
  double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
  double roll = std::atan2(sinr_cosp, cosr_cosp);

  double sinp = 2.0 * (w * y - z * x);
  double pitch = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI / 2.0, sinp)
                                         : std::asin(sinp);

  double siny_cosp = 2.0 * (w * z + x * y);
  double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  return {roll, pitch, yaw};
}

} // namespace bluerov2_controller
