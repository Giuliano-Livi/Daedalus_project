// OpenVINS odometry (/ov_msckf/odomimu: pose of the ZED IMU in OpenVINS' "global" frame) -> /vo/odom
// (pose of base_link), the interface of rgbd_odometry used by vision_to_px4 and the evaluation scripts.
//
// - IMU -> base_link: fixed transform from TF (base_link -> zed_imu_link, static, sim_bringup.launch.py).
// - Frame vo_odom: OpenVINS' global frame (gravity aligned, heading = the IMU's at initialization) moved so
//   that base_link starts at the origin, like rgbd_odometry. vision_to_px4 aligns the heading to north.
// - Rate: OpenVINS publishes at the IMU rate (250 Hz, propagated state); forwarded at `rate` Hz (default
//   15, the camera rate rgbd_odometry had), what vision_to_px4 and PX4's external vision expect.
// - Covariance: OpenVINS gives position (global) + orientation (error in the IMU frame); moved to
//   base_link with the lever arm IMU -> base_link: dp_B = dp_I - R_GI [r]x dth, dth_B = R_BI dth.

#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using nav_msgs::msg::Odometry;

namespace
{
Eigen::Matrix3d skew(const Eigen::Vector3d & v)
{
  Eigen::Matrix3d s;
  s << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  return s;
}
}  // namespace

class OpenvinsOdom : public rclcpp::Node
{
public:
  OpenvinsOdom()
  : Node("openvins_odom")
  {
    rate_ = declare_parameter<double>("rate", 15.0);
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "zed_imu_link");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "vo_odom");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    pub_ = create_publisher<Odometry>("/vo/odom", 10);
    sub_ = create_subscription<Odometry>(
      "/ov_msckf/odomimu", 50, [this](Odometry::ConstSharedPtr msg) {on_odom(*msg);});
  }

private:
  // base_link -> IMU, from the static TF (once)
  bool lookup_imu()
  {
    if (r_bi_) {return true;}
    try {
      const auto t = tf_buffer_->lookupTransform(base_frame_, imu_frame_, tf2::TimePointZero);
      const auto & q = t.transform.rotation;
      const auto & p = t.transform.translation;
      r_bi_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
      p_ib_ = Eigen::Vector3d(p.x, p.y, p.z);   // IMU position in base_link
      RCLCPP_INFO(get_logger(), "%s in %s: (%.3f, %.3f, %.3f) m", imu_frame_.c_str(), base_frame_.c_str(),
        p.x, p.y, p.z);
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for TF %s -> %s: %s",
        base_frame_.c_str(), imu_frame_.c_str(), e.what());
      return false;
    }
  }

  void on_odom(const Odometry & in)
  {
    const double t = in.header.stamp.sec + in.header.stamp.nanosec * 1e-9;
    if (last_t_ && t - *last_t_ < 1.0 / rate_ - 1e-3) {return;}
    if (!lookup_imu()) {return;}
    last_t_ = t;

    const auto & q = in.pose.pose.orientation;
    const auto & p = in.pose.pose.position;
    const Eigen::Matrix3d r_gi = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
    const Eigen::Vector3d p_gi(p.x, p.y, p.z);
    // base_link in the IMU frame: r = R_IB p_BinI, with p_BinI = -R_BI^T p_IinB
    const Eigen::Matrix3d r_ib = r_bi_->transpose();
    const Eigen::Vector3d r = -(r_ib * p_ib_);
    const Eigen::Matrix3d r_gb = r_gi * r_ib.transpose();     // R_GB = R_GI R_IB
    Eigen::Vector3d p_gb = p_gi + r_gi * r;
    if (!origin_) {origin_ = p_gb;}
    p_gb -= *origin_;

    Odometry out;
    out.header.stamp = in.header.stamp;
    out.header.frame_id = odom_frame_;
    out.child_frame_id = base_frame_;
    const Eigen::Quaterniond q_gb(r_gb);
    out.pose.pose.position.x = p_gb.x();
    out.pose.pose.position.y = p_gb.y();
    out.pose.pose.position.z = p_gb.z();
    out.pose.pose.orientation.w = q_gb.w();
    out.pose.pose.orientation.x = q_gb.x();
    out.pose.pose.orientation.y = q_gb.y();
    out.pose.pose.orientation.z = q_gb.z();

    // twist in base_link: v_B = R_BI (v_I + w x r), w_B = R_BI w (OpenVINS: both in the IMU frame)
    const auto & lv = in.twist.twist.linear;
    const auto & av = in.twist.twist.angular;
    const Eigen::Vector3d w(av.x, av.y, av.z);
    const Eigen::Vector3d v_b = *r_bi_ * (Eigen::Vector3d(lv.x, lv.y, lv.z) + w.cross(r));
    const Eigen::Vector3d w_b = *r_bi_ * w;
    out.twist.twist.linear.x = v_b.x();
    out.twist.twist.linear.y = v_b.y();
    out.twist.twist.linear.z = v_b.z();
    out.twist.twist.angular.x = w_b.x();
    out.twist.twist.angular.y = w_b.y();
    out.twist.twist.angular.z = w_b.z();

    Eigen::Matrix<double, 6, 6> cov, j = Eigen::Matrix<double, 6, 6>::Identity();
    for (int i = 0; i < 6; ++i) {
      for (int k = 0; k < 6; ++k) {cov(i, k) = in.pose.covariance[6 * i + k];}
    }
    j.block<3, 3>(0, 3) = -r_gi * skew(r);
    j.block<3, 3>(3, 3) = *r_bi_;
    cov = j * cov * j.transpose();
    for (int i = 0; i < 6; ++i) {
      for (int k = 0; k < 6; ++k) {
        out.pose.covariance[6 * i + k] = cov(i, k);
        out.twist.covariance[6 * i + k] = in.twist.covariance[6 * i + k];
      }
    }
    pub_->publish(out);
  }

  double rate_ = 15.0;
  std::string base_frame_, imu_frame_, odom_frame_;
  std::optional<Eigen::Matrix3d> r_bi_;
  Eigen::Vector3d p_ib_ = Eigen::Vector3d::Zero();
  std::optional<Eigen::Vector3d> origin_;
  std::optional<double> last_t_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OpenvinsOdom>());
  rclcpp::shutdown();
  return 0;
}
