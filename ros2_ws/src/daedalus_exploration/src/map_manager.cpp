#include "daedalus_exploration/map_manager.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <unordered_set>
#include <utility>

#include <std_msgs/msg/color_rgba.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>

namespace daedalus_exploration
{

namespace
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

octomap::Pointcloud transformed(const octomap::Pointcloud & points, const Eigen::Isometry3d & t)
{
  octomap::Pointcloud out;
  out.reserve(points.size());
  for (const auto & p : points) {
    const Eigen::Vector3d q = t * Eigen::Vector3d(p.x(), p.y(), p.z());
    out.push_back(static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z()));
  }
  return out;
}

double rotation_angle(const Eigen::Isometry3d & t)
{
  return Eigen::AngleAxisd(t.rotation()).angle();
}

// blue (low) -> green -> red (high) between lo and hi
std_msgs::msg::ColorRGBA height_color(double z, double lo, double hi)
{
  const double t = std::clamp((z - lo) / std::max(hi - lo, 1e-6), 0.0, 1.0);
  std_msgs::msg::ColorRGBA c;
  c.r = static_cast<float>(std::clamp(2 * t - 1, 0.0, 1.0));
  c.g = static_cast<float>(1 - std::abs(2 * t - 1));
  c.b = static_cast<float>(std::clamp(1 - 2 * t, 0.0, 1.0));
  c.a = 1.0f;
  return c;
}

// one CUBE_LIST per tree depth (the leaves of a pruned octree have different sizes), as octomap_server
MarkerArray voxel_markers(
  const octomap::OcTree & tree, bool occupied, const MapParams & params,
  const std::string & frame, const rclcpp::Time & stamp)
{
  const unsigned max_depth = tree.getTreeDepth();
  MarkerArray out;
  out.markers.resize(max_depth + 1);
  for (unsigned d = 0; d <= max_depth; ++d) {
    Marker & m = out.markers[d];
    m.header.frame_id = frame;
    m.header.stamp = stamp;
    m.ns = occupied ? "occupied" : "free";
    m.id = static_cast<int>(d);
    m.type = Marker::CUBE_LIST;
    m.scale.x = m.scale.y = m.scale.z = tree.getNodeSize(d);
    m.pose.orientation.w = 1.0;
    m.color.r = 0.2f;     // free: light transparent green (occupied: per-voxel colours by height)
    m.color.g = 0.9f;
    m.color.b = 0.3f;
    m.color.a = occupied ? 1.0f : 0.1f;
  }
  for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
    if (tree.isNodeOccupied(*it) != occupied) {continue;}
    Marker & m = out.markers[it.getDepth()];
    geometry_msgs::msg::Point p;
    p.x = it.getX();
    p.y = it.getY();
    p.z = it.getZ();
    m.points.push_back(p);
    if (occupied) {m.colors.push_back(height_color(p.z, params.z_min, params.z_max));}
  }
  for (Marker & m : out.markers) {
    m.action = m.points.empty() ? Marker::DELETE : Marker::ADD;
  }
  return out;
}

}  // namespace

MapManager::MapManager(rclcpp::Node & node)
: node_(node), last_inserted_(0, 0, node.get_clock()->get_clock_type())
{
  MapParams p;
  p.resolution = node.declare_parameter<double>("map.resolution", 0.2);        // m
  p.max_range = node.declare_parameter<double>("map.max_range", 5.0);          // m, as Grid/RangeMax
  // exploration volume: z from the floor (map z = 0 at the spawn point, on the floor), x/y optional
  p.z_min = node.declare_parameter<double>("map.z_min", 0.0);                  // m
  p.z_max = node.declare_parameter<double>("map.z_max", 5.0);                  // m
  const auto box = node.declare_parameter<std::vector<double>>("map.xy_box", std::vector<double>{});
  if (box.size() == 4) {
    p.xy_box = std::array<double, 4>{box[0], box[1], box[2], box[3]};
  } else if (!box.empty()) {
    RCLCPP_WARN(node.get_logger(), "map.xy_box needs [x_min, x_max, y_min, y_max]: ignored (unbounded)");
  }
  map_ = std::make_unique<ExplorationMap>(p);

  map_frame_ = node.declare_parameter<std::string>("map.map_frame", "map");
  odom_frame_ = node.declare_parameter<std::string>("map.odom_frame", "odom");
  // 672x376 / 4 = 168x94 rays per image, as point_cloud_xyz for RTAB-Map
  decimation_ = std::max(1, static_cast<int>(node.declare_parameter<int>("map.decimation", 4)));
  const double insert_rate = node.declare_parameter<double>("map.insert_rate", 3.0);   // Hz
  rebuild_translation_ = node.declare_parameter<double>("map.rebuild_translation", 0.3);
  rebuild_rotation_ = node.declare_parameter<double>("map.rebuild_rotation", 5.0) * M_PI / 180.0;
  // a cloud is stored for the rebuilds only if the camera moved since the last one: hovering adds nothing
  store_translation_ = node.declare_parameter<double>("map.store_translation", 0.2);
  store_rotation_ = node.declare_parameter<double>("map.store_rotation", 10.0) * M_PI / 180.0;
  store_voxel_ = node.declare_parameter<double>("map.store_voxel", 0.1);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node.get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // only the latest image is kept: the timer inserts at insert_rate, not at the camera's 15 Hz
  depth_sub_ = node.create_subscription<sensor_msgs::msg::Image>(
    node.declare_parameter<std::string>("map.depth_topic", "/camera/depth/image_raw"), 5,
    [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {depth_ = msg;});
  info_sub_ = node.create_subscription<sensor_msgs::msg::CameraInfo>(
    node.declare_parameter<std::string>("map.camera_info_topic", "/camera/depth/camera_info"), 5,
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {info_ = *msg;});
  // RTAB-Map graph: new keyframes (odometry pose) + optimized poses of all keyframes, at every update
  map_data_sub_ = node.create_subscription<rtabmap_msgs::msg::MapData>(
    node.declare_parameter<std::string>("map.map_data_topic", "/rtabmap/mapData"), 10,
    [this](const rtabmap_msgs::msg::MapData & msg) {on_map_data(msg);});

  occupied_pub_ = node.create_publisher<MarkerArray>("/exploration/occupied", 1);
  free_pub_ = node.create_publisher<MarkerArray>("/exploration/free", 1);

  using std::chrono::duration;
  insert_timer_ = rclcpp::create_timer(&node, node.get_clock(),
    std::chrono::duration_cast<std::chrono::nanoseconds>(duration<double>(1.0 / insert_rate)),
    [this]() {insert_depth();});
  rebuild_timer_ = rclcpp::create_timer(&node, node.get_clock(), std::chrono::seconds(1),
    [this]() {check_rebuild();});
  marker_timer_ = rclcpp::create_timer(&node, node.get_clock(), std::chrono::seconds(1),
    [this]() {publish_markers();});
}

void MapManager::insert_depth()
{
  if (!depth_ || !info_) {return;}
  const sensor_msgs::msg::Image & img = *depth_;
  const rclcpp::Time stamp(img.header.stamp, last_inserted_.get_clock_type());
  if (stamp <= last_inserted_) {return;}                   // already inserted (no new image)

  const bool mm = img.encoding == "16UC1";
  if (!mm && img.encoding != "32FC1") {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 5000,
      "depth encoding %s not supported (32FC1 or 16UC1)", img.encoding.c_str());
    return;
  }
  // camera pose at the image time: odometry (odom -> base_link -> camera) and SLAM correction (map -> odom)
  Eigen::Isometry3d odom_cam, map_odom;
  try {
    odom_cam = tf2::transformToEigen(tf_buffer_->lookupTransform(odom_frame_, img.header.frame_id, stamp));
    map_odom = tf2::transformToEigen(tf_buffer_->lookupTransform(map_frame_, odom_frame_, stamp));
  } catch (const tf2::TransformException & e) {
    RCLCPP_DEBUG(node_.get_logger(), "no TF for the depth image: %s", e.what());
    return;                                                // retried at the next tick
  }

  // points in the camera frame
  const double fx = info_->k[0], fy = info_->k[4], cx = info_->k[2], cy = info_->k[5];
  const int h = static_cast<int>(img.height), w = static_cast<int>(img.width);
  octomap::Pointcloud cam_points;
  cam_points.reserve(static_cast<size_t>((h / decimation_ + 1) * (w / decimation_ + 1)));
  for (int r = 0; r < h; r += decimation_) {
    const uint8_t * row = &img.data[static_cast<size_t>(r) * img.step];
    for (int c = 0; c < w; c += decimation_) {
      float z;
      if (mm) {
        uint16_t v;
        std::memcpy(&v, row + 2 * c, 2);
        z = v * 0.001f;
      } else {
        std::memcpy(&z, row + 4 * c, 4);
      }
      // NaN = no measurement (hole, or beyond the camera's range): nothing known along that ray.
      // Valid points beyond map.max_range clear the ray up to max_range (ExplorationMap::insert)
      if (!std::isfinite(z) || z <= 0.0f) {continue;}
      cam_points.push_back(static_cast<float>((c - cx) * z / fx), static_cast<float>((r - cy) * z / fy), z);
    }
  }
  const Eigen::Isometry3d map_cam = map_odom * odom_cam;
  map_->insert(transformed(cam_points, map_cam), map_cam.translation());
  store_cloud(stamp.seconds(), odom_cam, cam_points);
  last_inserted_ = stamp;
}

void MapManager::store_cloud(
  double stamp, const Eigen::Isometry3d & odom_cam, const octomap::Pointcloud & cam_points)
{
  if (last_stored_odom_cam_) {
    const Eigen::Isometry3d motion = last_stored_odom_cam_->inverse() * odom_cam;
    if (motion.translation().norm() < store_translation_ && rotation_angle(motion) < store_rotation_) {
      return;
    }
  }
  // one point per store_voxel cube (21 bits per axis: +-100 km at 0.1 m)
  auto cloud = std::make_shared<StoredCloud>();
  cloud->stamp = stamp;
  cloud->odom_cam = odom_cam;
  std::unordered_set<uint64_t> seen;
  seen.reserve(cam_points.size());
  for (const auto & p : cam_points) {
    uint64_t key = 0;
    for (int a = 0; a < 3; ++a) {
      key = (key << 21) | (static_cast<uint64_t>(std::floor(p(a) / store_voxel_) + (1 << 20)) & 0x1FFFFF);
    }
    if (seen.insert(key).second) {cloud->points.push_back(p);}
  }
  stored_points_ += cloud->points.size();
  clouds_.push_back(std::move(cloud));
  last_stored_odom_cam_ = odom_cam;
}

void MapManager::on_map_data(const rtabmap_msgs::msg::MapData & msg)
{
  // the new keyframes come with their odometry pose; the graph has the optimized pose of all of them
  for (const auto & node : msg.nodes) {
    Eigen::Isometry3d odom_base;
    tf2::fromMsg(node.pose, odom_base);
    keyframe_odom_[node.id] = {node.stamp, odom_base};
  }
  keyframe_map_.clear();
  for (size_t i = 0; i < msg.graph.poses_id.size() && i < msg.graph.poses.size(); ++i) {
    Eigen::Isometry3d map_base;
    tf2::fromMsg(msg.graph.poses[i], map_base);
    keyframe_map_[msg.graph.poses_id[i]] = map_base;
  }
}

void MapManager::check_rebuild()
{
  if (rebuild_.valid()) {                                  // redrawing: swap the map in when done
    if (rebuild_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {finish_rebuild();}
    return;
  }
  Eigen::Isometry3d map_odom;
  try {
    map_odom = tf2::transformToEigen(tf_buffer_->lookupTransform(map_frame_, odom_frame_, tf2::TimePointZero));
  } catch (const tf2::TransformException &) {
    return;                                                // rtabmap not running yet
  }
  if (!reference_map_odom_) {
    reference_map_odom_ = map_odom;                        // the map starts with this correction
    return;
  }
  // SLAM correction since the last build: the voxels inserted before it are misplaced by about this much
  const Eigen::Isometry3d delta = reference_map_odom_->inverse() * map_odom;
  const double translation = delta.translation().norm();
  const double rotation = rotation_angle(delta);
  if (translation < rebuild_translation_ && rotation < rebuild_rotation_) {return;}

  // map -> odom correction of every keyframe of the latest graph, sorted by time
  std::vector<Correction> corrections;
  for (const auto & [id, map_base] : keyframe_map_) {
    const auto it = keyframe_odom_.find(id);
    if (it != keyframe_odom_.end()) {
      corrections.push_back({it->second.first, map_base * it->second.second.inverse()});
    }
  }
  if (corrections.empty() || clouds_.empty()) {
    RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 10000,
      "SLAM correction %.2f m / %.1f deg but no keyframes from %s yet: map not redrawn",
      translation, rotation * 180.0 / M_PI, map_data_sub_->get_topic_name());
    return;
  }
  std::sort(corrections.begin(), corrections.end(),
    [](const Correction & a, const Correction & b) {return a.stamp < b.stamp;});

  RCLCPP_INFO(node_.get_logger(),
    "SLAM correction %.2f m / %.1f deg: redrawing the map from %zu clouds, %zu keyframes",
    translation, rotation * 180.0 / M_PI, clouds_.size(), corrections.size());
  reference_map_odom_ = map_odom;
  rebuild_clouds_ = clouds_.size();
  rebuild_start_ = std::chrono::steady_clock::now();
  // background thread, on copies: the stored clouds are never modified, the new map is private to it
  rebuild_ = std::async(std::launch::async,
      [params = map_->params(), clouds = clouds_, corrections = std::move(corrections)]() {
        auto map = std::make_unique<ExplorationMap>(params);
        for (const auto & cloud : clouds) {
          // keyframe nearest in time
          auto it = std::lower_bound(corrections.begin(), corrections.end(), cloud->stamp,
          [](const Correction & c, double t) {return c.stamp < t;});
          if (it == corrections.end() ||
          (it != corrections.begin() && cloud->stamp - std::prev(it)->stamp < it->stamp - cloud->stamp))
          {
            it = std::prev(it);
          }
          const Eigen::Isometry3d map_cam = it->map_odom * cloud->odom_cam;
          map->insert(transformed(cloud->points, map_cam), map_cam.translation());
        }
        return map;
      });
}

void MapManager::finish_rebuild()
{
  auto map = rebuild_.get();
  // clouds stored while redrawing: placed with the current correction
  for (size_t i = rebuild_clouds_; i < clouds_.size(); ++i) {
    const Eigen::Isometry3d map_cam = *reference_map_odom_ * clouds_[i]->odom_cam;
    map->insert(transformed(clouds_[i]->points, map_cam), map_cam.translation());
  }
  const MapStats before = map_->stats();
  const MapStats after = map->stats();
  map_ = std::move(map);
  ++rebuilds_;
  const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - rebuild_start_).count();
  RCLCPP_INFO(node_.get_logger(),
    "map redrawn in %.1f s (%zu clouds, %.1f M points stored): free %.0f -> %.0f m3, occupied %.0f -> %.0f m3",
    seconds, clouds_.size(), stored_points_ * 1e-6, before.free, after.free, before.occupied, after.occupied);
}

void MapManager::publish_markers()
{
  const auto stamp = node_.now();
  // the free voxels are many: built only if someone looks at them
  if (occupied_pub_->get_subscription_count() > 0) {
    occupied_pub_->publish(voxel_markers(map_->tree(), true, map_->params(), map_frame_, stamp));
  }
  if (free_pub_->get_subscription_count() > 0) {
    free_pub_->publish(voxel_markers(map_->tree(), false, map_->params(), map_frame_, stamp));
  }
}

}  // namespace daedalus_exploration
