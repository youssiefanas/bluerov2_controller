#ifndef BLUEROV2_CONTROLLER__ROV_CONTROLLER_NODE_HPP_
#define BLUEROV2_CONTROLLER__ROV_CONTROLLER_NODE_HPP_

#include <array>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <mavros_msgs/msg/override_rc_in.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/stream_rate.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

namespace bluerov2_controller
{

/**
 * @brief ROS 2 node for controlling a BlueROV2 via MAVROS.
 *
 * Handles joystick input, processes IMU and depth telemetry, manages
 * arm/disarm and servo commands (lights, camera tilt), and publishes
 * RC override PWM values for the ROV's 6 DOF. Supports switching between
 * ArduSub flight modes (Manual, Stabilize, Depth Hold, Position Hold)
 * via the MAVROS set_mode service.
 */
class RovControllerNode : public rclcpp::Node
{
public:
  explicit RovControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  /**
   * @brief ArduSub flight modes set on the Pixhawk flight controller.
   *
   * - MANUAL:     Raw RC passthrough, no stabilization.
   * - STABILIZE:  Auto-levels roll and pitch; pilot controls yaw, heave, surge, sway.
   * - DEPTH_HOLD: Stabilize + automatic depth holding via pressure sensor.
   * - POSHOLD:    Holds depth and horizontal position (requires DVL or positioning input).
   */
  enum class FlightMode { MANUAL, STABILIZE, DEPTH_HOLD, POSHOLD };

  /**
   * @brief Returns the ArduSub custom_mode number string for MAVROS set_mode service.
   * @param mode The desired flight mode.
   * @return Mode number as string ("19"=Manual, "0"=Stabilize, "2"=Depth Hold, "16"=PosHold).
   */
  static std::string flight_mode_to_string(FlightMode mode);

  /**
   * @brief Returns a human-readable name for a flight mode.
   * @param mode The flight mode.
   * @return Mode name string (e.g., "MANUAL", "STABILIZE", "DEPTH_HOLD", "POSHOLD").
   */
  static const char * flight_mode_name(FlightMode mode);

  /**
   * @brief Sends a MAVROS set_mode request to switch the ArduSub flight mode on the FC.
   * @param mode The desired flight mode.
   */
  void set_flight_mode(FlightMode mode);

  /**
   * @brief Joystick button callback for mode switching, arm/disarm, and servo control.
   * @param msg Joy message containing button states and axis values.
   */
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  /**
   * @brief Velocity command callback that maps joystick axes to motor PWM.
   *
   * RC override is sent in all flight modes. The flight controller interprets
   * the PWM inputs differently depending on the active mode (e.g., in Depth Hold
   * the heave channel controls depth change rate rather than raw thruster output).
   * @param msg Twist message from teleop_twist_joy with linear (surge, sway, heave) and angular (roll, pitch, yaw) velocities.
   */
  void vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Auxiliary velocity callback. Latches the latest aux Twist + timestamp;
   * it is summed onto the joystick cmd_vel inside vel_callback and decays to zero
   * after aux_timeout_sec_ of silence.
   */
  void vel_aux_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief IMU data callback that converts quaternion orientation to Euler angles and publishes angular state.
   * @param msg Imu message with orientation quaternion and angular velocity from MAVROS.
   */
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

  /**
   * @brief Barometer relative altitude callback for depth tracking and publishing.
   * @param msg Float64 message with relative altitude from MAVROS global_position plugin.
   */
  void rel_alt_callback(const std_msgs::msg::Float64::SharedPtr msg);

  /**
   * @brief Static pressure callback from the Bar30 sensor via MAVROS.
   * @param msg FluidPressure message with absolute pressure in Pascals.
   */
  void pressure_callback(const sensor_msgs::msg::FluidPressure::SharedPtr msg);

  /**
   * @brief Arms or disarms the vehicle motors via MAV_CMD_COMPONENT_ARM_DISARM (command 400).
   * @param arm True to arm, false to disarm.
   */
  void arm_disarm(bool arm);

  /**
   * @brief Sets the MAVROS telemetry stream rate for sensor data from the flight controller.
   * @param rate Desired stream rate in Hz (e.g., 25 for IMU data).
   */
  void set_stream_rate(int rate);

  /**
   * @brief Configures the mount to MAVLINK targeting mode via MAV_CMD_DO_MOUNT_CONFIGURE (204).
   */
  void set_mount_mode();

  /**
   * @brief Commands the camera mount pitch via MAV_CMD_DO_MOUNT_CONTROL (205).
   * @param pitch_deg Desired pitch angle in degrees.
   */
  void send_mount_pitch(double pitch_deg);

  /**
   * @brief Publishes an RC override message with PWM values for all 6 DOF plus camera servos.
   * @param pitch   Channel 0: pitch (nose up/down) PWM [1100-1900].
   * @param roll    Channel 1: roll (tilt left/right) PWM [1100-1900].
   * @param heave   Channel 2: heave (vertical up/down) PWM [1100-1900].
   * @param yaw     Channel 3: yaw (rotate left/right) PWM [1100-1900].
   * @param surge   Channel 4: surge (forward/backward) PWM [1100-1900].
   * @param sway    Channel 5: sway (lateral left/right) PWM [1100-1900].
   */
  void set_override_rcin(int pitch, int roll, int heave, int yaw,
                         int surge, int sway);

  /**
   * @brief Runs a hardware initialization test by flashing the light and sweeping the camera servo.
   */
  void run_initialization_test();

  /**
   * @brief Maps a normalized velocity value [-1, 1] to a PWM value [pwm_min_, pwm_max_].
   * @param value Normalized velocity (-1.0 = full reverse, 0.0 = stop, 1.0 = full forward).
   * @return Clamped PWM integer value.
   */
  int map_value_scale_saturate(double value) const;

  /**
   * @brief Converts a quaternion orientation to Euler angles (roll, pitch, yaw).
   * @param x Quaternion x component.
   * @param y Quaternion y component.
   * @param z Quaternion z component.
   * @param w Quaternion w component.
   * @return Array of {roll, pitch, yaw} in radians.
   */
  static std::array<double, 3> quaternion_to_euler(double x, double y,
                                                    double z, double w);

  // Subscribers
  /// Joystick raw input for button events (arm, disarm, mode switch, servo control)
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  /// Velocity commands from teleop_twist_joy mapping joystick axes to surge/sway/heave/roll/pitch/yaw
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  /// Auxiliary velocity input summed onto cmd_vel before PWM (e.g. plain_pid_controller output)
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_aux_sub_;
  /// IMU orientation and angular velocity from MAVROS (Pixhawk flight controller)
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  /// Barometer-based relative altitude from MAVROS for depth measurement
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rel_alt_sub_;
  /// Absolute static pressure (Pa) from the Bar30 sensor via MAVROS
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr pressure_sub_;

  // Publishers
  /// RC override to send PWM commands to MAVROS for all 8 channels (6 DOF + 2 camera servos)
  rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr rc_override_pub_;
  /// Euler angles (roll, pitch, yaw) in degrees relative to the orientation at startup
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr angle_degree_pub_;
  /// Depth value from the barometric pressure sensor
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr depth_pub_;
  /// Absolute static pressure (Pa) republished from MAVROS
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pressure_pub_;
  /// Angular velocity (p, q, r) in rad/s from the IMU gyroscope
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr angular_velocity_pub_;

  // Service clients (persistent, created once at startup)
  /// MAVROS CommandLong service for arm/disarm (cmd 400) and servo control (cmd 183)
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr cmd_client_;
  /// MAVROS StreamRate service to set telemetry data rate from the flight controller
  rclcpp::Client<mavros_msgs::srv::StreamRate>::SharedPtr stream_rate_client_;
  /// MAVROS SetMode service to switch ArduSub flight modes (Manual, Stabilize, Depth Hold, PosHold)
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

  // State
  FlightMode flight_mode_{FlightMode::MANUAL};
  bool armed_{false};

  // Auxiliary cmd_vel (e.g. from plain_pid_controller) latched between joystick ticks.
  geometry_msgs::msg::Twist cmd_vel_aux_;
  rclcpp::Time cmd_vel_aux_stamp_;
  bool cmd_vel_aux_seen_{false};
  double aux_timeout_sec_{0.5};

  // Servo state
  double tilt_angle_;  // camera mount pitch in degrees

  // Previous button states for edge detection
  bool prev_btn_tilt_up_{false};
  bool prev_btn_tilt_down_{false};
  bool prev_btn_tilt_reset_{false};

  // Parameters
  int pwm_neutral_;
  int pwm_min_;
  int pwm_max_;
  int pwm_scale_;
  double tilt_angle_min_;
  double tilt_angle_max_;
  double tilt_angle_initial_;
  double tilt_angle_step_;
  int btn_arm_;
  int btn_disarm_;
  int btn_manual_mode_;
  int btn_stabilize_mode_;
  int btn_depth_hold_mode_;
  int btn_poshold_mode_;
  int btn_cam_tilt_up_;
  int btn_cam_tilt_down_;
  int btn_cam_tilt_reset_;
  double service_timeout_sec_;
};

}  // namespace bluerov2_controller

#endif  // BLUEROV2_CONTROLLER__ROV_CONTROLLER_NODE_HPP_
