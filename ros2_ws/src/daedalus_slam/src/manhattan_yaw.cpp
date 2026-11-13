// Manhattan-world heading: the drone's heading relative to the warehouse's orthogonal structure (racks,
// uprights, walls), measured on every depth image -> /manhattan/yaw (daedalus_interfaces/ManhattanYaw).
// vision_to_px4 (mode vio) uses it as an absolute heading reference against the visual-inertial yaw drift.
//
// 1. Surface normals on a grid (every `stride` px) of the depth image, from the 3D points of the neighbours
//    at +-stride (cross product), only closer than max_range (the depth noise grows with the distance) and
//    away from depth edges (neighbours not on one plane: second difference of the inverse depth).
// 2. Normals -> base_link (static TF) -> levelled with roll and pitch of the visual odometry (/vo/odom,
//    gravity-aligned): only the near-horizontal ones are kept (vertical surfaces; floor and ceiling out).
// 3. Their angles phi_i (FLU, counter-clockwise from forward) are 4-fold symmetric (4 directions 90 deg
//    apart): yaw = atan2(sum w sin 4phi, sum w cos 4phi) / 4, quality = length of that mean (0..1).
// Floor (`floor`, -> /floor/height, daedalus_interfaces/FloorHeight): points on near-vertical normals (upward
// surfaces) below the drone, up to floor_max_range (the forward camera sees the floor only from ~4.4 m ahead
// at 3 m of altitude). Their heights below base_link (levelled) go in a histogram: the floor is the LOWEST
// surface with enough points (box tops and pallets are horizontal too, but higher), refined with the mean of
// the points within floor_window of it.

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <daedalus_interfaces/msg/floor_height.hpp>
#include <daedalus_interfaces/msg/manhattan_yaw.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using daedalus_interfaces::msg::FloorHeight;
using daedalus_interfaces::msg::ManhattanYaw;
using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;

namespace
{
double stamp_s(const builtin_interfaces::msg::Time & t) {return t.sec + t.nanosec * 1e-9;}
}  // namespace

class ManhattanYawNode : public rclcpp::Node
{
public:
  ManhattanYawNode()
  : Node("manhattan_yaw")
  {
    stride_ = declare_parameter<int>("stride", 4);                       // px
    min_range_ = declare_parameter<double>("min_range", 0.3);            // m
    max_range_ = declare_parameter<double>("max_range", 5.0);            // m, as Vis/MaxDepth of RTAB-Map
    edge_ = declare_parameter<double>("edge", 0.05);                     // relative non-planarity = edge
    // a normal is "horizontal" if it is less than this from the horizontal plane
    horizontal_tol_ = declare_parameter<double>("horizontal_tol", 15.0) * M_PI / 180.0;
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    floor_ = declare_parameter<bool>("floor", true);
    floor_max_range_ = declare_parameter<double>("floor_max_range", 7.0);   // m, depth noise model limit
    // a normal is "vertical" (upward surface) if it is less than this from the vertical
    floor_tol_ = declare_parameter<double>("floor_tol", 10.0) * M_PI / 180.0;
    floor_below_ = declare_parameter<double>("floor_below", 0.2);           // m, points at least this below base_link
    floor_bin_ = declare_parameter<double>("floor_bin", 0.02);              // m, histogram bin
    floor_min_bin_ = declare_parameter<int>("floor_min_bin", 40);           // points in bin +- 1 for a surface
    floor_window_ = declare_parameter<double>("floor_window", 0.04);        // m, refinement window

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    pub_ = create_publisher<ManhattanYaw>("/manhattan/yaw", 10);
    if (floor_) {floor_pub_ = create_publisher<FloorHeight>("/floor/height", 10);}
    info_sub_ = create_subscription<CameraInfo>(
      "/camera/depth/camera_info", 5, [this](CameraInfo::ConstSharedPtr msg) {info_ = *msg;});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/vo/odom", 10, [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {on_odom(*msg);});
    depth_sub_ = create_subscription<Image>(
      "/camera/depth/image_raw", 5, [this](Image::ConstSharedPtr msg) {on_depth(*msg);});
  }

private:
  // roll/pitch only (yaw removed) of base_link in the visual frame, kept for ~1 s
  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    const auto & q = msg.pose.pose.orientation;
    if (q.w == 0.0 && q.x == 0.0 && q.y == 0.0 && q.z == 0.0) {return;}
    const Eigen::Matrix3d r = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
    const double yaw = std::atan2(r(1, 0), r(0, 0));
    const Eigen::Matrix3d level = Eigen::AngleAxisd(-yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * r;
    tilts_.emplace_back(stamp_s(msg.header.stamp), level);
    while (tilts_.size() > 30) {tilts_.pop_front();}
  }

  std::optional<Eigen::Matrix3d> tilt_at(double t) const
  {
    std::optional<Eigen::Matrix3d> best;
    double best_dt = 0.2;                                                // s, older = no measurement
    for (const auto & [ts, r] : tilts_) {
      if (std::abs(ts - t) < best_dt) {best_dt = std::abs(ts - t); best = r;}
    }
    return best;
  }

  bool lookup_camera(const std::string & frame)
  {
    if (r_bc_) {return true;}
    try {
      const auto t = tf_buffer_->lookupTransform(base_frame_, frame, tf2::TimePointZero);
      const auto & q = t.transform.rotation;
      const auto & p = t.transform.translation;
      r_bc_ = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix();
      t_bc_ = Eigen::Vector3d(p.x, p.y, p.z);                            // camera origin in base_link
      return true;
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for TF %s -> %s: %s",
        base_frame_.c_str(), frame.c_str(), e.what());
      return false;
    }
  }

  void on_depth(const Image & img)
  {
    if (!info_ || img.encoding != "32FC1" || img.is_bigendian) {return;}
    if (!lookup_camera(img.header.frame_id)) {return;}
    const double t = stamp_s(img.header.stamp);
    const auto tilt = tilt_at(t);
    if (!tilt) {return;}
    // camera -> levelled base_link (yaw left out: angles relative to the drone's forward axis)
    const Eigen::Matrix3d r = *tilt * *r_bc_;
    const double fx = info_->k[0], fy = info_->k[4], cx = info_->k[2], cy = info_->k[5];
    const int w = static_cast<int>(img.width), h = static_cast<int>(img.height), s = stride_;
    auto depth = [&](int u, int v) {
        return reinterpret_cast<const float *>(&img.data[v * img.step])[u];
      };
    auto point = [&](int u, int v, double z) {
        return Eigen::Vector3d((u - cx) * z / fx, (v - cy) * z / fy, z);
      };
    const double range = floor_ ? std::max(max_range_, floor_max_range_) : max_range_;
    auto valid = [&](double z) {return std::isfinite(z) && z >= min_range_ && z <= range;};
    const double min_vertical = std::cos(floor_tol_);
    std::vector<double> heights;                                        // floor candidates below base_link

    const double max_vertical = std::sin(horizontal_tol_);
    double c = 0.0, sn = 0.0, wsum = 0.0;
    uint32_t n = 0;
    for (int v = s; v < h - s; v += s) {
      for (int u = s; u < w - s; u += s) {
        const double z = depth(u, v), zl = depth(u - s, v), zr = depth(u + s, v);
        const double zu = depth(u, v - s), zd = depth(u, v + s);
        if (!valid(z) || !valid(zl) || !valid(zr) || !valid(zu) || !valid(zd)) {continue;}
        // depth edge: the neighbours are not on the same plane. On a plane 1/z is linear in u and v, so its
        // second difference is 0 also for surfaces seen at a grazing angle (the racks along the aisle)
        if (std::abs(1 / zr + 1 / zl - 2 / z) > edge_ / z || std::abs(1 / zd + 1 / zu - 2 / z) > edge_ / z) {continue;}
        const Eigen::Vector3d dx = point(u + s, v, zr) - point(u - s, v, zl);
        const Eigen::Vector3d dy = point(u, v + s, zd) - point(u, v - s, zu);
        Eigen::Vector3d nrm = dx.cross(dy);
        const double len = nrm.norm();
        if (len < 1e-9) {continue;}
        nrm = r * (nrm / len);
        if (std::abs(nrm.z()) >= min_vertical) {
          // upward / downward surface: below the drone it can be the floor
          if (floor_ && z <= floor_max_range_) {
            const Eigen::Vector3d p = *tilt * (*r_bc_ * point(u, v, z) + t_bc_);
            if (p.z() < -floor_below_) {heights.push_back(-p.z());}
          }
          continue;
        }
        if (std::abs(nrm.z()) > max_vertical || z > max_range_) {continue;}   // slanted, or too far
        const double phi = std::atan2(nrm.y(), nrm.x());
        const double wt = nrm.x() * nrm.x() + nrm.y() * nrm.y();         // more weight to vertical surfaces
        c += wt * std::cos(4.0 * phi);
        sn += wt * std::sin(4.0 * phi);
        wsum += wt;
        ++n;
      }
    }
    if (floor_) {publish_floor(img, heights);}
    if (n == 0) {return;}
    ManhattanYaw out;
    out.header.stamp = img.header.stamp;
    out.header.frame_id = base_frame_;
    out.yaw = std::atan2(sn, c) / 4.0;
    out.quality = std::hypot(c, sn) / wsum;
    out.num_normals = n;
    pub_->publish(out);
  }

  // floor: lowest surface (largest height below base_link) whose bin and neighbours have floor_min_bin points
  void publish_floor(const Image & img, const std::vector<double> & h)
  {
    if (h.size() < static_cast<size_t>(floor_min_bin_)) {return;}
    const double hmax = *std::max_element(h.begin(), h.end());
    const int nb = static_cast<int>(hmax / floor_bin_) + 2;
    std::vector<int> hist(nb, 0);
    for (double x : h) {++hist[static_cast<int>(x / floor_bin_)];}
    int found = -1;
    for (int b = nb - 2; b >= 1; --b) {
      if (hist[b - 1] + hist[b] + hist[b + 1] >= floor_min_bin_) {found = b; break;}
    }
    if (found < 0) {return;}
    const double centre = (found + 0.5) * floor_bin_;
    double sum = 0.0, sum2 = 0.0;
    uint32_t cnt = 0;
    for (double x : h) {
      if (std::abs(x - centre) <= floor_window_) {sum += x; sum2 += x * x; ++cnt;}
    }
    if (cnt == 0) {return;}
    FloorHeight out;
    out.header.stamp = img.header.stamp;
    out.header.frame_id = base_frame_;
    out.height = sum / cnt;
    out.num_points = cnt;
    out.spread = std::sqrt(std::max(0.0, sum2 / cnt - out.height * out.height));
    floor_pub_->publish(out);
  }

  int stride_ = 4;
  double min_range_ = 0.3, max_range_ = 5.0, edge_ = 0.05, horizontal_tol_ = 0.26;
  std::string base_frame_;
  std::optional<CameraInfo> info_;
  std::optional<Eigen::Matrix3d> r_bc_;                                   // camera optical -> base_link
  Eigen::Vector3d t_bc_ = Eigen::Vector3d::Zero();
  bool floor_ = true;
  double floor_max_range_ = 7.0, floor_tol_ = 0.17, floor_below_ = 0.2, floor_bin_ = 0.02, floor_window_ = 0.04;
  int floor_min_bin_ = 40;
  rclcpp::Publisher<FloorHeight>::SharedPtr floor_pub_;
  std::deque<std::pair<double, Eigen::Matrix3d>> tilts_;                  // (stamp, roll/pitch rotation)
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<ManhattanYaw>::SharedPtr pub_;
  rclcpp::Subscription<CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<Image>::SharedPtr depth_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManhattanYawNode>());
  rclcpp::shutdown();
  return 0;
}
