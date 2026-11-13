// PX4 EKF odometry (/fmu/out/vehicle_odometry, NED/FRD) -> /odom + TF odom -> base_link (ENU/FLU).
//
// The odom frame is PX4's local frame turned to ENU, so it is aligned with the Gazebo world
// (x east, y north) and centred on the spawn, like the setpoints of trajectory_generator.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "daedalus_slam/frames.hpp"

using daedalus_slam::frd_to_flu;
using daedalus_slam::ned_to_enu;
using px4_msgs::msg::VehicleOdometry;

namespace
{

// non-finite components -> `value` (np.nan_to_num)
Eigen::Vector3d finite_or(const Eigen::Vector3d & v, double value)
{
  Eigen::Vector3d out;
  for (int i = 0; i < 3; ++i) {out[i] = std::isfinite(v[i]) ? v[i] : value;}
  return out;
}

template<typename A>
Eigen::Vector3d vec3(const A & a)
{
  return {a[0], a[1], a[2]};
}

}  // namespace

class Px4Odometry : public rclcpp::Node
{
public:
  Px4Odometry()
  : Node("px4_odometry")
  {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    tf_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    sub_ = create_subscription<VehicleOdometry>(
      "/fmu/out/vehicle_odometry", rclcpp::QoS(10).best_effort(),
      [this](const VehicleOdometry & msg) {update_odometry(msg);});
  }

private:
  void update_odometry(const VehicleOdometry & msg)
  {
    if (msg.pose_frame != VehicleOdometry::POSE_FRAME_NED) {return;}
    for (float c : msg.q) {
      if (!std::isfinite(c)) {return;}
    }
    // stamp with ROS (sim) time on arrival: same clock as the camera images
    const auto stamp = now();

    const Eigen::Vector3d p = ned_to_enu() * vec3(msg.position);
    // base_link (FLU) -> odom (ENU)
    const Eigen::Matrix3d r =
      ned_to_enu() * daedalus_slam::quat_to_rot(msg.q[0], msg.q[1], msg.q[2], msg.q[3]) * frd_to_flu();
    const Eigen::Vector4d q = daedalus_slam::rot_to_quat(r);

    // twist in base_link, as nav_msgs/Odometry expects
    Eigen::Vector3d v = vec3(msg.velocity);
    if (msg.velocity_frame == VehicleOdometry::VELOCITY_FRAME_NED) {
      v = r.transpose() * (ned_to_enu() * v);
    } else {                                                      // already in body FRD
      v = frd_to_flu() * v;
    }
    const Eigen::Vector3d w = finite_or(frd_to_flu() * vec3(msg.angular_velocity), 0.0);
    v = finite_or(v, 0.0);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = p.x();
    odom.pose.pose.position.y = p.y();
    odom.pose.pose.position.z = p.z();
    odom.pose.pose.orientation.w = q[0];
    odom.pose.pose.orientation.x = q[1];
    odom.pose.pose.orientation.y = q[2];
    odom.pose.pose.orientation.z = q[3];
    odom.twist.twist.linear.x = v.x();
    odom.twist.twist.linear.y = v.y();
    odom.twist.twist.linear.z = v.z();
    odom.twist.twist.angular.x = w.x();
    odom.twist.twist.angular.y = w.y();
    odom.twist.twist.angular.z = w.z();

    // covariances from the EKF variances (NED -> ENU swaps x/y); floor so RTAB-Map never
    // gets a zero (= infinitely trusted) covariance
    const Eigen::Vector3d pv = finite_or(vec3(msg.position_variance), 1.0);
    const Eigen::Vector3d ov = finite_or(vec3(msg.orientation_variance), 1.0);
    const Eigen::Vector3d vv = finite_or(vec3(msg.velocity_variance), 1.0);
    const double pose_var[6] = {
      std::max(pv[1], 1e-4), std::max(pv[0], 1e-4), std::max(pv[2], 1e-4),
      std::max(ov[0], 1e-5), std::max(ov[1], 1e-5), std::max(ov[2], 1e-5)};
    const double twist_var[6] = {
      std::max(vv[1], 1e-4), std::max(vv[0], 1e-4), std::max(vv[2], 1e-4), 1e-3, 1e-3, 1e-3};
    for (int i = 0; i < 6; ++i) {
      odom.pose.covariance[i * 7] = pose_var[i];
      odom.twist.covariance[i * 7] = twist_var[i];
    }
    odom_pub_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header = odom.header;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = p.x();
      tf.transform.translation.y = p.y();
      tf.transform.translation.z = p.z();
      tf.transform.rotation = odom.pose.pose.orientation;
      tf_pub_->sendTransform(tf);
    }
  }

  const std::string odom_frame_ = "odom";
  const std::string base_frame_ = "base_link";
  const bool publish_tf_ = true;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
  rclcpp::Subscription<VehicleOdometry>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4Odometry>());
  rclcpp::shutdown();
  return 0;
}
