// Offboard control: streams the setpoints of /drone/setpoint (ENU, FlightSetpoint) to PX4 (NED,
// TrajectorySetpoint), holds the position when they stop, switches PX4 to offboard and arms it.

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>

#include <daedalus_interfaces/msg/flight_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using daedalus_interfaces::msg::FlightSetpoint;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

namespace
{
constexpr float NaN = std::numeric_limits<float>::quiet_NaN();
}

class OffboardControl : public rclcpp::Node
{
public:
  OffboardControl()
  : Node("offboard_control")
  {
    // SLAM correction: the setpoints are meant in rtabmap's map frame; PX4 flies in its own frame
    // (odom), which drifts without GPS. map -> odom (rtabmap's TF, changed by every loop closure)
    // converts them, applied gradually so a loop closure does not make the setpoint jump
    slam_correction_ = declare_parameter<bool>("slam_correction", false);
    if (slam_correction_) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    // quality of service def
    const auto px4_output_qos = rclcpp::QoS(10).best_effort();
    const auto setpoint_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

    // publishers
    control_mode_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    setpoint_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    // subscribers
    position_sub_ = create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", px4_output_qos,
      [this](const VehicleLocalPosition & msg) {update_position(msg);});
    status_sub_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status_v4", px4_output_qos,
      [this](const VehicleStatus & msg) {update_status(msg);});
    setpoint_sub_ = create_subscription<FlightSetpoint>(
      "/drone/setpoint", setpoint_qos,
      [this](const FlightSetpoint & msg) {update_setpoint(msg);});

    // 10Hz loop
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(100),
        [this]() {offboard_control_loop();});
  }

private:
  // functions
  void publish_hold(uint64_t timestamp)
  {
    if (!hold_position_) {hold_position_ = current_position_;}
    OffboardControlMode control_mode;
    control_mode.timestamp = timestamp;
    control_mode.position = true;
    control_mode_pub_->publish(control_mode);

    TrajectorySetpoint setpoint;
    setpoint.timestamp = timestamp;
    setpoint.position = *hold_position_;
    setpoint.velocity = {NaN, NaN, NaN};
    setpoint.acceleration = {NaN, NaN, NaN};
    setpoint.jerk = {NaN, NaN, NaN};
    setpoint.yaw = NaN;
    setpoint.yawspeed = NaN;
    setpoint_pub_->publish(setpoint);
  }

  void publish_setpoint(uint64_t timestamp)
  {
    OffboardControlMode control_mode;
    control_mode.timestamp = timestamp;
    control_mode.position = true;
    control_mode_pub_->publish(control_mode);

    TrajectorySetpoint setpoint;
    setpoint.timestamp = timestamp;
    setpoint.position = setpoint_pos_;
    setpoint.velocity = setpoint_vel_;
    setpoint.acceleration = setpoint_acc_;
    setpoint.jerk = {NaN, NaN, NaN};
    setpoint.yaw = setpoint_yaw_;
    setpoint.yawspeed = setpoint_yaw_rate_;
    setpoint_pub_->publish(setpoint);
  }

  void publish_vehicle_command(uint64_t timestamp, uint32_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    VehicleCommand msg;
    msg.timestamp = timestamp;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
  }

  // callbacks
  void update_position(const VehicleLocalPosition & msg)
  {
    if (msg.xy_valid && msg.z_valid) {current_position_ = {msg.x, msg.y, msg.z};}
  }

  void update_status(const VehicleStatus & msg)
  {
    offboard_active_ = msg.nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
    armed_ = msg.arming_state == VehicleStatus::ARMING_STATE_ARMED;
  }

  void update_setpoint(const FlightSetpoint & msg)
  {
    setpoint_raw_ = msg;
    apply_setpoint();
    last_setpoint_ns_ = now().nanoseconds();
  }

  void apply_setpoint()
  {
    const auto & msg = *setpoint_raw_;
    const auto & p = msg.position;
    const auto & v = msg.velocity;
    const auto & a = msg.acceleration;
    // map frame -> PX4's frame (inverse of map -> odom), still ENU
    const double cx = corr_[0], cy = corr_[1], cyaw = corr_[2];
    const double c = std::cos(-cyaw), s = std::sin(-cyaw);
    auto rot = [c, s](double x, double y) {return std::array<double, 2>{c * x - s * y, s * x + c * y};};
    const auto pp = rot(p.x - cx, p.y - cy);
    const auto vv = rot(v.x, v.y);
    const auto aa = rot(a.x, a.y);
    // ENU (ROS) -> NED (PX4): swap x/y, flip z
    setpoint_pos_ = {static_cast<float>(pp[1]), static_cast<float>(pp[0]), static_cast<float>(-p.z)};
    setpoint_vel_ = {static_cast<float>(vv[1]), static_cast<float>(vv[0]), static_cast<float>(-v.z)};
    setpoint_acc_ = {static_cast<float>(aa[1]), static_cast<float>(aa[0]), static_cast<float>(-a.z)};
    setpoint_yaw_ = static_cast<float>(M_PI / 2 - (msg.yaw - cyaw));   // NaN stays NaN
    setpoint_yaw_rate_ = static_cast<float>(-msg.yaw_rate);
    have_setpoint_ = true;
  }

  void update_correction(double dt)
  {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;                           // no SLAM (yet): no correction
    }
    const auto & q = t.transform.rotation;
    const std::array<double, 3> target = {
      t.transform.translation.x, t.transform.translation.y,
      std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))};
    const double step_xy = corr_rate_[0] * dt, step_yaw = corr_rate_[1] * dt;
    double dx = target[0] - corr_[0], dy = target[1] - corr_[1];
    const double d = std::hypot(dx, dy);
    if (d > step_xy) {
      dx = dx * step_xy / d;
      dy = dy * step_xy / d;
    }
    const double dyaw = std::atan2(std::sin(target[2] - corr_[2]), std::cos(target[2] - corr_[2]));
    corr_ = {corr_[0] + dx, corr_[1] + dy, corr_[2] + std::max(-step_yaw, std::min(step_yaw, dyaw))};
    if (std::hypot(target[0] - corr_logged_[0], target[1] - corr_logged_[1]) > 0.2 ||
      std::abs(target[2] - corr_logged_[2]) > 2.0 * M_PI / 180.0)
    {
      corr_logged_ = target;
      RCLCPP_INFO(get_logger(), "new SLAM correction (loop closure): (%.2f, %.2f) m, %.1f deg, applied gradually",
        target[0], target[1], target[2] * 180.0 / M_PI);
    }
  }

  void offboard_control_loop()
  {
    if (!current_position_) {return;}
    const int64_t now_ns = now().nanoseconds();
    const uint64_t timestamp = now_ns / 1000;
    if (slam_correction_) {
      update_correction(0.1);
      if (setpoint_raw_) {apply_setpoint();}
    }

    // 1) ALWAYS stream: generator setpoint if fresh, otherwise hold
    const bool fresh = have_setpoint_ && now_ns - last_setpoint_ns_ < 500000000;
    if (fresh) {
      publish_setpoint(timestamp);
    } else {
      publish_hold(timestamp);
    }

    // 2) handshake (only after the stream exists)
    if (++setpoint_count_ < 10) {return;}
    if (!offboard_active_ && now_ns - last_mode_request_ns_ > 1000000000) {
      publish_vehicle_command(timestamp, VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
      last_mode_request_ns_ = now_ns;
    } else if (offboard_active_ && !armed_ && now_ns - last_arm_request_ns_ > 1000000000) {
      publish_vehicle_command(timestamp, VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
      last_arm_request_ns_ = now_ns;
    }
  }

  std::optional<std::array<float, 3>> current_position_;
  std::optional<std::array<float, 3>> hold_position_;
  int64_t last_setpoint_ns_ = 0;
  int64_t last_mode_request_ns_ = 0;
  int64_t last_arm_request_ns_ = 0;

  bool have_setpoint_ = false;
  std::array<float, 3> setpoint_pos_{}, setpoint_vel_{}, setpoint_acc_{};
  float setpoint_yaw_ = 0.0f, setpoint_yaw_rate_ = 0.0f;
  int setpoint_count_ = 0;

  bool offboard_active_ = false;
  bool armed_ = false;

  bool slam_correction_ = false;
  std::array<double, 3> corr_ = {0.0, 0.0, 0.0};            // applied correction map -> odom: x, y (ENU, m), yaw (rad)
  const std::array<double, 2> corr_rate_ = {0.3, 5.0 * M_PI / 180.0};   // max change: m/s, rad/s
  std::optional<FlightSetpoint> setpoint_raw_;              // last setpoint from the generator (map frame, ENU)
  std::array<double, 3> corr_logged_ = {0.0, 0.0, 0.0};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<OffboardControlMode>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<FlightSetpoint>::SharedPtr setpoint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardControl>());
  rclcpp::shutdown();
  return 0;
}
