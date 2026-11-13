// Visual odometry (/odom, odom -> base_link in FLU) -> PX4 external vision, the inverse of px4_odometry.
//
// rgbd_odometry's odom frame is base_link's start pose, with an arbitrary heading: it is rotated to
// PX4's north and sent as POSE_FRAME_NED, position + body velocity (EKF2_EV_CTRL=5, start.sh --no-gps).
// PX4's heading is ~10 deg wrong on the ground and converges in flight, so the rotation is set twice:
//   - provisional, on the ground;
//   - final, ALIGN_DELAY after takeoff, keeping the current position (no jump).
// Positions are always sent with a large std (POS_STD), so they bound PX4's position uncertainty
// without overriding its magnetometer heading.
// POSE_FRAME_FRD (EKF2 re-aligning the frame continuously from the visual vs magnetic heading) is not
// used: it rotated every position about the start, tens of metres of error 30 m away.
//
// mode "vio" (OpenVINS, odom:=openvins, start.sh --vio): PX4 has no magnetometer and takes position,
// velocity and yaw from vision (EKF2_EV_CTRL=15). The visual frame is rotated once, at the first
// message, so that the drone's heading is `dock_heading` (NED heading of the dock the drone starts
// from, part of the warehouse map: 0 = north, the spawn of the simulation), with the origin at the
// start. Position, velocity (OpenVINS' body velocity) and attitude are sent with OpenVINS' variances
// (EKF2 lower-bounds them, EKF2_EV*_NOISE). `dropouts` ([start, duration, ...] in s after takeoff)
// stops sending for a while: test of the inertial + drag prediction of EKF2 without vision.
// With `manhattan`, the heading drift of the visual-inertial odometry is corrected with /manhattan/yaw
// (manhattan_yaw: heading relative to the warehouse's orthogonal structure, known modulo 90 deg; the axes
// are at `manhattan_heading`, NED): measurements with good quality, not during fast turns and close to the
// current heading (mw_gate; on the ground before takeoff up to 45 deg, to fix the dock heading) rotate the
// visual frame towards them, gradually (time constant mw_tau) and about the current position (no jump).
// With `floor`, the height drift is corrected with /floor/height (manhattan_yaw: height above the floor). The
// floor level in the map frame is taken from the first fl_ref_samples measurements after takeoff (on the
// ground the floor is not below base_link, which is at the feet); then height = floor level + measured
// height, approached gradually (fl_tau, at most fl_max_rate) by moving the visual frame vertically; differences
// above fl_gate are rejected (a box top taken for the floor).

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <daedalus_interfaces/msg/floor_height.hpp>
#include <daedalus_interfaces/msg/manhattan_yaw.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "daedalus_slam/frames.hpp"

using daedalus_slam::frd_to_flu;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleOdometry;

namespace
{

// provisional alignment on the ground: PX4's heading must stay within HEADING_SPREAD for HEADING_WINDOW
constexpr double HEADING_WINDOW = 2.0;      // s
constexpr double HEADING_SPREAD = 1.0;      // deg
constexpr double HEADING_TIMEOUT = 15.0;    // s, then align anyway
// final alignment: after ALIGN_DELAY above ALIGN_HEIGHT, when PX4's heading has converged in flight
constexpr double ALIGN_HEIGHT = 1.5;        // m above the start
constexpr double ALIGN_DELAY = 8.0;         // s
// minimum position std sent to PX4, for the whole flight: the positions only have to bound PX4's
// position uncertainty (it refuses to arm / fails safe above COM_POS_FS_EPH = 5 m). Sent with their
// real std (~cm) they override the magnetometer: PX4's heading followed the visual heading drift
// (+3 deg in 20 s), the drone drifted 1.4 m sideways and hit a shelf. Velocity does the short-term work
constexpr double POS_STD = 2.0;             // m
// velocity from positions over VEL_WINDOW: rgbd_odometry's twist is the frame-to-frame motion / 66 ms,
// so a few cm of depth noise per frame become +-0.5 m/s (and tens of m/s after a tracking loss)
constexpr double VEL_WINDOW = 0.3;          // s
constexpr double VEL_STD = 0.25;            // m/s, reported to EKF2 (~noise of the windowed difference)
constexpr double VEL_MAX = 3.0;             // m/s, above this the sample is a glitch (trajectory max is 1 m/s)

constexpr double deg(double rad) {return rad * 180.0 / M_PI;}
constexpr double rad(double deg) {return deg * M_PI / 180.0;}

Eigen::Matrix3d rot_z(double a)
{
  return Eigen::AngleAxisd(a, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

double yaw_of(const Eigen::Matrix3d & r)
{
  return std::atan2(r(1, 0), r(0, 0));
}

}  // namespace

class VisionToPx4 : public rclcpp::Node
{
public:
  VisionToPx4()
  : Node("vision_to_px4")
  {
    mode_ = declare_parameter<std::string>("mode", "rgbd");
    if (mode_ != "rgbd" && mode_ != "vio") {throw std::runtime_error("mode must be 'rgbd' or 'vio'");}
    dock_heading_ = rad(declare_parameter<double>("dock_heading", 0.0));
    dropouts_ = declare_parameter<std::vector<double>>("dropouts", std::vector<double>{});
    if (dropouts_.size() % 2) {throw std::runtime_error("dropouts: [start, duration, ...]");}
    manhattan_ = declare_parameter<bool>("manhattan", false);
    manhattan_heading_ = rad(declare_parameter<double>("manhattan_heading", 0.0));
    mw_min_quality_ = declare_parameter<double>("mw_min_quality", 0.45);  // typical 0.6 in the aisles (eval_manhattan.py)
    mw_min_normals_ = declare_parameter<int>("mw_min_normals", 500);     // quality ~1 with few normals: unreliable
    mw_gate_ = rad(declare_parameter<double>("mw_gate", 5.0));             // deg in flight
    mw_max_rate_ = rad(declare_parameter<double>("mw_max_rate", 10.0));    // deg/s
    mw_tau_ = declare_parameter<double>("mw_tau", 3.0);                    // s
    // safety: the heading drift of the VIO is ~0.05 deg/s, an anomaly must not turn the frame quickly
    mw_max_corr_rate_ = rad(declare_parameter<double>("mw_max_corr_rate", 0.3));   // deg/s
    floor_ = declare_parameter<bool>("floor", false);
    fl_gate_ = declare_parameter<double>("fl_gate", 0.3);                  // m
    fl_tau_ = declare_parameter<double>("fl_tau", 2.0);                    // s
    fl_max_rate_ = declare_parameter<double>("fl_max_rate", 0.05);         // m/s
    fl_min_points_ = declare_parameter<int>("fl_min_points", 40);    // 15 Hz in flight, RMS ~0.1 m (eval_floor.py)
    fl_max_spread_ = declare_parameter<double>("fl_max_spread", 0.1);      // m
    fl_ref_samples_ = declare_parameter<int>("fl_ref_samples", 20);
    if (mode_ == "vio" && floor_) {
      fl_sub_ = create_subscription<daedalus_interfaces::msg::FloorHeight>(
        "/floor/height", 10, [this](const daedalus_interfaces::msg::FloorHeight & msg) {on_floor(msg);});
      fl_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/floor/correction", 10);
    }
    if (mode_ == "vio" && manhattan_) {
      mw_sub_ = create_subscription<daedalus_interfaces::msg::ManhattanYaw>(
        "/manhattan/yaw", 10, [this](const daedalus_interfaces::msg::ManhattanYaw & msg) {on_manhattan(msg);});
      mw_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/manhattan/correction", 10);
    }
    pub_ = create_publisher<VehicleOdometry>("/fmu/in/vehicle_visual_odometry", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, [this](const nav_msgs::msg::Odometry & msg) {forward(msg);});
    px4_sub_ = create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", rclcpp::QoS(10).best_effort(),
      [this](const VehicleLocalPosition & msg) {update_px4(msg);});
  }

private:
  double now_s() {return now().nanoseconds() * 1e-9;}

  void update_px4(const VehicleLocalPosition & msg)
  {
    if (!std::isfinite(msg.heading)) {return;}
    px4_heading_ = msg.heading;
    const double t = now_s();
    if (!align_) {
      headings_.emplace_back(t, msg.heading);
      while (t - headings_.front().first > HEADING_WINDOW) {headings_.pop_front();}
    }
    if (msg.z_valid && msg.z < -ALIGN_HEIGHT) {
      if (!airborne_since_) {airborne_since_ = t;}
      if (!takeoff_) {takeoff_ = t;}
    } else {
      airborne_since_.reset();
    }
  }

  std::optional<double> stable_heading()
  {
    const double t = now_s();
    if (!start_) {start_ = t;}
    if (headings_.empty() || t - headings_.front().first < 0.9 * HEADING_WINDOW) {return std::nullopt;}
    // unwrap, then spread (max - min) and mean
    std::vector<double> h;
    for (const auto & [_, a] : headings_) {
      double v = a;
      if (!h.empty()) {
        const double d = v - h.back();
        v = h.back() + std::atan2(std::sin(d), std::cos(d));
      }
      h.push_back(v);
    }
    const auto [lo, hi] = std::minmax_element(h.begin(), h.end());
    const double spread = *hi - *lo;
    if (spread > rad(HEADING_SPREAD) && t - *start_ < HEADING_TIMEOUT) {return std::nullopt;}
    if (spread > rad(HEADING_SPREAD)) {
      RCLCPP_WARN(get_logger(), "PX4 heading not stable, aligning anyway");
    }
    double sum = 0.0;
    for (double v : h) {sum += v;}
    return sum / h.size();
  }

  bool update_alignment(const Eigen::Matrix3d & r_vo, const Eigen::Vector3d & p_vo)
  {
    if (!align_) {
      const auto heading = stable_heading();
      if (!heading) {return false;}
      align_ = rot_z(*heading - yaw_of(r_vo));
      RCLCPP_INFO(get_logger(), "provisional alignment on the ground: %.1f deg",
        deg(*heading - yaw_of(r_vo)));
    } else if (!final_ && airborne_since_ && now_s() - *airborne_since_ > ALIGN_DELAY && last_ned_) {
      const Eigen::Matrix3d fresh = rot_z(px4_heading_ - yaw_of(r_vo));
      const double change = deg(yaw_of(fresh) - yaw_of(*align_));
      align_ = fresh;
      // keep the current position: only the motion from now on is rotated differently
      offset_ = *last_ned_ - fresh * p_vo;
      offset_.z() = 0.0;
      history_.clear();
      final_ = true;
      RCLCPP_INFO(get_logger(), "final alignment in flight: %.1f deg (%+.1f deg from the ground)",
        deg(yaw_of(fresh)), change);
    }
    return true;
  }

  // body-frame velocity from the position change over ~VEL_WINDOW; both in the vision frame,
  // so it does not depend on the visual heading drift (PX4 rotates it with its own heading)
  std::optional<Eigen::Vector3d> velocity(double stamp, const Eigen::Vector3d & p_ned, const Eigen::Matrix3d & r)
  {
    history_.emplace_back(stamp, p_ned);
    while (stamp - history_.front().first > VEL_WINDOW) {history_.pop_front();}
    const auto & [t0, p0] = history_.front();
    if (stamp - t0 < 0.5 * VEL_WINDOW) {return std::nullopt;}
    const Eigen::Vector3d v = (p_ned - p0) / (stamp - t0);
    if (v.norm() > VEL_MAX) {
      history_.clear();
      return std::nullopt;
    }
    return r.transpose() * v;
  }

  // inside one of the `dropouts` windows (s after takeoff)
  bool in_dropout()
  {
    bool in = false;
    if (takeoff_) {
      const double t = now_s() - *takeoff_;
      for (size_t i = 0; i < dropouts_.size(); i += 2) {
        in = in || (t >= dropouts_[i] && t < dropouts_[i] + dropouts_[i + 1]);
      }
    }
    if (in != dropout_) {
      RCLCPP_WARN(get_logger(), in ? "dropout test: vision NOT sent to PX4" : "dropout test over: vision sent again");
      dropout_ = in;
    }
    return in;
  }

  void forward(const nav_msgs::msg::Odometry & odom)
  {
    if (mode_ == "vio") {
      forward_vio(odom);
      return;
    }
    double pose_var[6];
    bool any = false;
    double max_var = 0.0;
    for (int i = 0; i < 6; ++i) {
      pose_var[i] = odom.pose.covariance[i * 7];
      any = any || pose_var[i] != 0.0;
      max_var = std::max(max_var, pose_var[i]);
    }
    // rgbd_odometry publishes a null odometry (zero covariance) while it has lost track
    if (!any || max_var >= 9999) {
      if (!lost_) {RCLCPP_WARN(get_logger(), "visual odometry lost, nothing sent to PX4");}
      lost_ = true;
      history_.clear();   // no velocity across the gap
      return;
    }
    if (lost_) {
      // rgbd_odometry restarts from its latest pose (Odom/ResetCountdown), so the pose stays
      // continuous: reset_counter is left alone, bumping it makes EKF2 reset its position
      lost_ = false;
      RCLCPP_INFO(get_logger(), "visual odometry back");
    }

    const auto & p = odom.pose.pose.position;
    const auto & o = odom.pose.pose.orientation;
    // base_link (FRD) -> odom (FRD)
    const Eigen::Matrix3d r_vo = frd_to_flu() * daedalus_slam::quat_to_rot(o.w, o.x, o.y, o.z) * frd_to_flu();
    const Eigen::Vector3d p_vo = frd_to_flu() * Eigen::Vector3d(p.x, p.y, p.z);
    if (!update_alignment(r_vo, p_vo)) {return;}
    const Eigen::Matrix3d r = *align_ * r_vo;
    const Eigen::Vector3d p_ned = *align_ * p_vo + offset_;
    last_ned_ = p_ned;
    const double stamp = rclcpp::Time(odom.header.stamp).nanoseconds() * 1e-9;
    const auto v = velocity(stamp, p_ned, r);
    const auto & w = odom.twist.twist.angular;
    const Eigen::Vector3d w_frd = frd_to_flu() * Eigen::Vector3d(w.x, w.y, w.z);
    const Eigen::Vector4d q = daedalus_slam::rot_to_quat(r);
    const float nan = std::numeric_limits<float>::quiet_NaN();

    VehicleOdometry msg;
    msg.timestamp = now().nanoseconds() / 1000;
    msg.timestamp_sample = static_cast<uint64_t>(stamp * 1e6);
    msg.pose_frame = VehicleOdometry::POSE_FRAME_NED;
    msg.velocity_frame = VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
    for (int i = 0; i < 3; ++i) {
      msg.position[i] = p_ned[i];
      // NaN = no velocity in this sample (start, after a gap or a glitch); EKF2 skips it
      msg.velocity[i] = v ? static_cast<float>((*v)[i]) : nan;
      msg.angular_velocity[i] = w_frd[i];
      msg.position_variance[i] = std::max(pose_var[i], POS_STD * POS_STD);
      msg.orientation_variance[i] = pose_var[3 + i];
      msg.velocity_variance[i] = VEL_STD * VEL_STD;
    }
    for (int i = 0; i < 4; ++i) {msg.q[i] = q[i];}
    pub_->publish(msg);
  }

  void forward_vio(const nav_msgs::msg::Odometry & odom)
  {
    const auto & c = odom.pose.covariance;
    if (c[0] == 0.0 && c[7] == 0.0) {return;}   // no estimate yet
    const auto & p = odom.pose.pose.position;
    const auto & o = odom.pose.pose.orientation;
    // base_link (FRD) -> visual frame (z down)
    const Eigen::Matrix3d r_vo = frd_to_flu() * daedalus_slam::quat_to_rot(o.w, o.x, o.y, o.z) * frd_to_flu();
    const Eigen::Vector3d p_vo = frd_to_flu() * Eigen::Vector3d(p.x, p.y, p.z);
    if (!align_) {
      align_ = rot_z(dock_heading_ - yaw_of(r_vo));
      offset_ = -(*align_ * p_vo);
      RCLCPP_INFO(get_logger(), "vio: visual frame aligned to the dock heading %.1f deg (rotation %.1f deg)",
        deg(dock_heading_), deg(dock_heading_ - yaw_of(r_vo)));
    }
    const double t_odom = rclcpp::Time(odom.header.stamp).nanoseconds() * 1e-9;
    yaws_.emplace_back(t_odom, yaw_of(r_vo));
    while (yaws_.size() > 2 && t_odom - yaws_.front().first > 2.0) {yaws_.pop_front();}
    zs_.emplace_back(t_odom, p_vo.z());
    while (zs_.size() > 2 && t_odom - zs_.front().first > 2.0) {zs_.pop_front();}
    const Eigen::Matrix3d r = *align_ * r_vo;
    const Eigen::Vector3d p_ned = *align_ * p_vo + offset_;
    last_p_vo_ = p_vo;
    last_ned_ = p_ned;
    // takeoff also from the visual height (no PX4 in a bag replay: offline check of the corrections)
    if (!takeoff_ && -p_ned.z() > ALIGN_HEIGHT) {takeoff_ = now_s();}
    if (in_dropout()) {return;}
    const auto & lv = odom.twist.twist.linear;
    const auto & w = odom.twist.twist.angular;
    const Eigen::Vector3d v_frd = frd_to_flu() * Eigen::Vector3d(lv.x, lv.y, lv.z);
    const Eigen::Vector3d w_frd = frd_to_flu() * Eigen::Vector3d(w.x, w.y, w.z);
    const Eigen::Vector4d q = daedalus_slam::rot_to_quat(r);
    // horizontal variances: the visual frame is rotated, x and y mix -> the larger of the two for both
    const double var_h = std::max(c[0], c[7]);
    const double pos_var[3] = {var_h, var_h, c[14]};

    VehicleOdometry msg;
    msg.timestamp = now().nanoseconds() / 1000;
    msg.timestamp_sample = rclcpp::Time(odom.header.stamp).nanoseconds() / 1000;
    msg.pose_frame = VehicleOdometry::POSE_FRAME_NED;
    msg.velocity_frame = VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
    for (int i = 0; i < 3; ++i) {
      msg.position[i] = p_ned[i];
      msg.velocity[i] = v_frd[i];
      msg.angular_velocity[i] = w_frd[i];
      msg.position_variance[i] = pos_var[i];
      msg.orientation_variance[i] = c[21 + 7 * i];
      msg.velocity_variance[i] = odom.twist.covariance[7 * i];
    }
    for (int i = 0; i < 4; ++i) {msg.q[i] = q[i];}
    pub_->publish(msg);
  }

  // heading of the visual odometry (visual frame) at time t and its rate, from the recent history
  bool vo_yaw_at(double t, double & yaw, double & rate) const
  {
    for (size_t i = 1; i < yaws_.size(); ++i) {
      const auto & [t0, y0] = yaws_[i - 1];
      const auto & [t1, y1] = yaws_[i];
      if (t0 <= t && t <= t1 && t1 > t0) {
        const double d = std::atan2(std::sin(y1 - y0), std::cos(y1 - y0));
        yaw = y0 + d * (t - t0) / (t1 - t0);
        rate = d / (t1 - t0);
        return true;
      }
    }
    // the depth image is often a bit newer than the last pose (15 Hz each, not synchronized): extrapolated
    // with the last rate up to 150 ms
    if (yaws_.size() >= 2 && t > yaws_.back().first && t - yaws_.back().first < 0.15) {
      const auto & [t0, y0] = yaws_[yaws_.size() - 2];
      const auto & [t1, y1] = yaws_.back();
      if (t1 > t0) {
        rate = std::atan2(std::sin(y1 - y0), std::cos(y1 - y0)) / (t1 - t0);
        yaw = y1 + rate * (t - t1);
        return true;
      }
    }
    return false;
  }

  // height (up, map frame) sent to PX4 at time t: visual z (down) interpolated + vertical offset
  bool height_at(double t, double & h) const
  {
    for (size_t i = 1; i < zs_.size(); ++i) {
      const auto & [t0, z0] = zs_[i - 1];
      const auto & [t1, z1] = zs_[i];
      if (t0 <= t && t <= t1 && t1 > t0) {
        h = -(z0 + (z1 - z0) * (t - t0) / (t1 - t0) + offset_.z());
        return true;
      }
    }
    if (!zs_.empty() && t > zs_.back().first && t - zs_.back().first < 0.15) {
      h = -(zs_.back().second + offset_.z());
      return true;
    }
    return false;
  }

  void on_floor(const daedalus_interfaces::msg::FloorHeight & msg)
  {
    if (!align_ || !takeoff_) {return;}
    const double t = rclcpp::Time(msg.header.stamp).nanoseconds() * 1e-9;
    double h_est = 0.0;
    const char * reject = nullptr;
    double err = std::numeric_limits<double>::quiet_NaN(), step = 0.0;
    if (static_cast<int>(msg.num_points) < fl_min_points_ || msg.spread > fl_max_spread_) {
      reject = "quality";
    } else if (!height_at(t, h_est)) {
      reject = "no odometry at that time";
    } else if (!floor_level_) {
      // floor level in the map frame, from the first measurements in flight
      fl_ref_.push_back(h_est - msg.height);
      if (static_cast<int>(fl_ref_.size()) >= fl_ref_samples_) {
        std::vector<double> v = fl_ref_;
        std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
        floor_level_ = v[v.size() / 2];
        RCLCPP_INFO(get_logger(), "floor: level %.2f m in the map frame (%zu measurements)", *floor_level_, v.size());
      }
      reject = "reference";
    } else {
      err = *floor_level_ + msg.height - h_est;
      if (std::abs(err) > fl_gate_) {reject = "gate";}
    }
    if (!reject) {
      const double dt = fl_last_ ? std::clamp(t - *fl_last_, 0.0, 0.5) : 0.1;
      fl_last_ = t;
      step = std::clamp(err * std::min(1.0, dt / fl_tau_), -fl_max_rate_ * dt, fl_max_rate_ * dt);
      offset_.z() -= step;                       // NED: up by step
      if (last_ned_) {last_ned_->z() -= step;}
      ++fl_used_;
      fl_total_ += step;
    } else {
      ++fl_rejected_[reject];
    }
    if (fl_pub_) {
      geometry_msgs::msg::Vector3Stamped c;
      c.header = msg.header;
      c.vector.x = err;
      c.vector.y = step;
      c.vector.z = reject ? 0.0 : 1.0;
      fl_pub_->publish(c);
    }
    std::string rej;
    for (const auto & [k, n] : fl_rejected_) {rej += " " + k + "=" + std::to_string(n);}
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
      "floor: %d used, total correction %+.2f m, last error %+.2f m; rejected:%s",
      fl_used_, fl_total_, err, rej.c_str());
  }

  void on_manhattan(const daedalus_interfaces::msg::ManhattanYaw & msg)
  {
    if (!align_ || !last_p_vo_ || !last_ned_) {return;}
    const double t = rclcpp::Time(msg.header.stamp).nanoseconds() * 1e-9;
    double yaw_vo = 0.0, rate = 0.0;
    const char * reject = nullptr;
    double err = std::numeric_limits<double>::quiet_NaN();
    if (msg.quality < mw_min_quality_ || static_cast<int>(msg.num_normals) < mw_min_normals_) {
      reject = "quality";
    } else if (!vo_yaw_at(t, yaw_vo, rate)) {
      reject = "no odometry at that time";
    } else if (std::abs(rate) > mw_max_rate_) {
      reject = "turning";
    } else {
      // heading (NED) known modulo 90 deg: axes heading + measured angle of an axis in the levelled body
      // frame (FLU, counter-clockwise -> clockwise heading: same sign once taken modulo 90 deg)
      const double estimate = yaw_of(*align_) + yaw_vo;
      const double d = manhattan_heading_ + msg.yaw - estimate;
      err = d - M_PI / 2 * std::floor((d + M_PI / 4) / (M_PI / 2));          // -> [-45, 45) deg
      const double gate = takeoff_ ? mw_gate_ : M_PI / 4;
      if (std::abs(err) > gate) {reject = "gate";}
    }
    double step = 0.0;
    if (!reject) {
      const double dt = mw_last_ ? std::clamp(t - *mw_last_, 0.0, 0.5) : 0.1;
      mw_last_ = t;
      // rate limit in flight only: on the ground the first lock to the warehouse axes can be quick
      const double max_step = takeoff_ ? mw_max_corr_rate_ * dt : M_PI;
      step = std::clamp(err * std::min(1.0, dt / mw_tau_), -max_step, max_step);
      // rotate the visual frame about the current position: no jump, the motion from now on is corrected
      align_ = rot_z(step) * *align_;
      offset_ = *last_ned_ - *align_ * *last_p_vo_;
      ++mw_used_;
      mw_total_ += step;
    } else {
      ++mw_rejected_[reject];
    }
    if (mw_pub_) {
      geometry_msgs::msg::Vector3Stamped c;
      c.header = msg.header;
      c.vector.x = deg(err);
      c.vector.y = deg(step);
      c.vector.z = reject ? 0.0 : 1.0;
      mw_pub_->publish(c);
    }
    std::string rej;
    for (const auto & [k, n] : mw_rejected_) {rej += " " + k + "=" + std::to_string(n);}
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 10000,
      "manhattan: %d used, total correction %+.2f deg, last error %+.2f deg; rejected:%s",
      mw_used_, deg(mw_total_), deg(err), rej.c_str());
  }

  rclcpp::Publisher<VehicleOdometry>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr px4_sub_;
  double px4_heading_ = 0.0;
  std::deque<std::pair<double, double>> headings_;          // (time [s], heading [rad]) on the ground
  std::optional<double> start_;
  std::optional<double> airborne_since_;
  std::optional<Eigen::Matrix3d> align_;                     // rotation odom (FRD) -> NED
  Eigen::Vector3d offset_ = Eigen::Vector3d::Zero();         // translation added after the rotation
  bool final_ = false;
  std::optional<Eigen::Vector3d> last_ned_;                  // last position sent, kept across the final alignment
  std::deque<std::pair<double, Eigen::Vector3d>> history_;   // (stamp [s], position NED) for the velocity
  bool lost_ = false;
  std::string mode_;
  double dock_heading_ = 0.0;                                // rad, NED heading at the start (vio)
  std::vector<double> dropouts_;                             // [start, duration, ...] s after takeoff
  std::optional<double> takeoff_;                            // first time above ALIGN_HEIGHT
  bool dropout_ = false;
  // Manhattan heading correction
  bool manhattan_ = false;
  double manhattan_heading_ = 0.0, mw_min_quality_ = 0.5, mw_gate_ = 0.09, mw_max_rate_ = 0.17, mw_tau_ = 3.0;
  double mw_max_corr_rate_ = 0.005;
  int mw_min_normals_ = 200;
  std::deque<std::pair<double, double>> yaws_;               // (stamp, heading of the visual odometry)
  std::optional<Eigen::Vector3d> last_p_vo_;
  std::optional<double> mw_last_;
  int mw_used_ = 0;
  double mw_total_ = 0.0;
  std::map<std::string, int> mw_rejected_;
  rclcpp::Subscription<daedalus_interfaces::msg::ManhattanYaw>::SharedPtr mw_sub_;
  // floor height correction
  bool floor_ = false;
  double fl_gate_ = 0.3, fl_tau_ = 2.0, fl_max_rate_ = 0.05, fl_max_spread_ = 0.1;
  int fl_min_points_ = 100, fl_ref_samples_ = 20;
  std::deque<std::pair<double, double>> zs_;                 // (stamp, visual z, down)
  std::vector<double> fl_ref_;
  std::optional<double> floor_level_, fl_last_;
  int fl_used_ = 0;
  double fl_total_ = 0.0;
  std::map<std::string, int> fl_rejected_;
  rclcpp::Subscription<daedalus_interfaces::msg::FloorHeight>::SharedPtr fl_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr fl_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr mw_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionToPx4>());
  rclcpp::shutdown();
  return 0;
}
