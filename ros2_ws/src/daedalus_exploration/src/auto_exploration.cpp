// Autonomous exploration (AEP, Selin et al. 2019): take off to 3 m from the spawn point, turn 360 deg in
// place so the camera sees all around (first map), then hover. The 3D map (unknown / free / occupied)
// is built by MapManager from the depth camera.
//
// State machine (10 Hz): wait_odom -> takeoff -> scan -> hover

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include <daedalus_interfaces/msg/flight_setpoint.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "daedalus_exploration/map_manager.hpp"

using daedalus_interfaces::msg::FlightSetpoint;

namespace
{

void quat_to_euler(double w, double x, double y, double z, double & roll, double & pitch, double & yaw)
{
  roll = std::atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  pitch = std::asin(std::max(-1.0, std::min(1.0, 2 * (w * y - z * x))));
  yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
}

}  // namespace

class AutoExploration : public rclcpp::Node
{
public:
  AutoExploration()
  : Node("auto_exploration")
  {
    // setpoints for offboard_control (do not run trajectory_generator together: same topic)
    setpoint_pub_ = create_publisher<FlightSetpoint>("/drone/setpoint", 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [this](const nav_msgs::msg::Odometry & msg) {update_odom(msg);});
    // 3D map: own OctoMap from the depth camera, rebuilt from rtabmap's after large SLAM corrections
    map_ = std::make_unique<daedalus_exploration::MapManager>(*this);

    // 10Hz loop, on the node's clock (simulation time with use_sim_time)
    timer_ = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(100), [this]() {exploration_loop();});
  }

private:
  // flight
  const double takeoff_altitude_ = 3.0;    // m
  const double takeoff_speed_ = 0.5;       // m/s: the target height rises gradually instead of a 3 m step
  const double dt_ = 0.1;                  // s, period of the main loop
  // 360 deg scan after the takeoff: rtabmap adds a map node once per second and the camera sees
  // 101 deg, so at <= 20 deg/s every direction is seen in several overlapping views; slow also
  // for the visual odometry (PX4 limit MC_YAWRATE_MAX 30 deg/s)
  const double scan_rate_max_ = 20.0 * M_PI / 180.0;          // rad/s, peak yaw rate
  // cosine profile, from rest to rest: yaw = yaw0 + 2pi (1 - cos(pi t / T)) / 2, peak rate pi^2 / T
  const double scan_duration_ = M_PI * M_PI / scan_rate_max_;   // ~28 s

  std::optional<std::array<double, 3>> current_position_;
  std::array<double, 3> current_orientation_ = {0.0, 0.0, 0.0};   // roll, pitch, yaw (rad)
  std::unique_ptr<daedalus_exploration::MapManager> map_;

  std::string state_ = "wait_odom";        // wait_odom -> takeoff -> scan -> hover
  std::array<double, 3> setpoint_ = {0.0, 0.0, 0.0};   // [x, y, z] being commanded (ENU, origin at the spawn point)
  double setpoint_yaw_ = 0.0;              // rad, ENU
  double scan_time_ = 0.0;                 // s since the scan started
  double scan_start_yaw_ = 0.0;

  rclcpp::Publisher<FlightSetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void update_odom(const nav_msgs::msg::Odometry & msg)
  {
    const auto & pos = msg.pose.pose.position;
    current_position_ = std::array<double, 3>{pos.x, pos.y, pos.z};
    const auto & q = msg.pose.pose.orientation;
    quat_to_euler(q.w, q.x, q.y, q.z, current_orientation_[0], current_orientation_[1], current_orientation_[2]);
  }

  // position + velocity feed-forward + yaw to offboard_control; acceleration not commanded (NaN)
  void publish_setpoint(double vx = 0.0, double vy = 0.0, double vz = 0.0, double yaw_rate = 0.0)
  {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    FlightSetpoint msg;
    msg.header.stamp = now();
    msg.header.frame_id = "map";
    msg.position.x = setpoint_[0];
    msg.position.y = setpoint_[1];
    msg.position.z = setpoint_[2];
    msg.velocity.x = vx;
    msg.velocity.y = vy;
    msg.velocity.z = vz;
    msg.acceleration.x = nan;
    msg.acceleration.y = nan;
    msg.acceleration.z = nan;
    msg.yaw = setpoint_yaw_;
    msg.yaw_rate = yaw_rate;
    setpoint_pub_->publish(msg);
  }

  void exploration_loop()
  {
    // main loop, called every 0.1 s: works on the latest data saved by the callbacks
    if (!current_position_) {return;}                              // no odometry yet
    const auto [x, y, z] = *current_position_;
    const double yaw = current_orientation_[2];

    if (state_ == "wait_odom") {
      // first odometry: the drone is on the ground at the spawn point (0, 0, 0); take off
      // vertically to (0, 0, takeoff_altitude) keeping the current heading
      setpoint_ = {0.0, 0.0, z};
      setpoint_yaw_ = yaw;
      state_ = "takeoff";
      RCLCPP_INFO(get_logger(), "takeoff to (0, 0, %g) m", takeoff_altitude_);
    }

    if (state_ == "takeoff") {
      if (setpoint_[2] < takeoff_altitude_) {                     // raise the target at takeoff_speed
        setpoint_[2] = std::min(takeoff_altitude_, setpoint_[2] + takeoff_speed_ * dt_);
        publish_setpoint(0.0, 0.0, takeoff_speed_);
      } else {
        publish_setpoint();
      }
      if (std::abs(z - takeoff_altitude_) < 0.1) {
        state_ = "scan";
        scan_time_ = 0.0;
        scan_start_yaw_ = setpoint_yaw_;
        RCLCPP_INFO(get_logger(), "takeoff done at (%.2f, %.2f, %.2f) m: 360 deg scan, %.0f s",
          x, y, z, scan_duration_);
      }
    } else if (state_ == "scan") {
      // turn in place (position held), yaw rate feed-forward = derivative of the profile
      scan_time_ = std::min(scan_time_ + dt_, scan_duration_);
      const double phase = M_PI * scan_time_ / scan_duration_;
      const double turned = 2 * M_PI * (1 - std::cos(phase)) / 2;
      const double rate = M_PI * M_PI / scan_duration_ * std::sin(phase);
      const double yaw_sp = scan_start_yaw_ + turned;
      setpoint_yaw_ = std::atan2(std::sin(yaw_sp), std::cos(yaw_sp));   // wrapped to [-pi, pi]
      publish_setpoint(0.0, 0.0, 0.0, rate);
      if (scan_time_ >= scan_duration_) {
        state_ = "hover";
        const auto stats = map_->map().stats();
        RCLCPP_INFO(get_logger(), "360 deg scan done: hovering. Map: free %.0f m3, occupied %.0f m3",
          stats.free, stats.occupied);
      }
    } else if (state_ == "hover") {
      publish_setpoint();                                         // keep streaming: offboard_control holds if no setpoint for 0.5 s
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutoExploration>());
  rclcpp::shutdown();
  return 0;
}
