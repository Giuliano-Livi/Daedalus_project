// ROS side of the exploration map:
//  - our own OctoMap from the depth image, ~3 Hz, posed with TF map -> camera
//  - every depth cloud is also stored (camera frame, thinned, with its odometry pose) when the camera
//    has moved enough since the last stored one
//  - when RTAB-Map corrects the drone pose a lot (TF map -> odom moves more than a threshold since the
//    last build) the old voxels are misplaced: the map is redrawn from the stored clouds, each placed
//    with the correction of the RTAB-Map keyframe nearest in time (/rtabmap/mapData: odometry pose and
//    optimized pose of every keyframe). Done in a background thread, swapped in when ready.
//    (RTAB-Map's own octomap has obstacles but no free space: it cannot be used for this)
//  - RViz markers of the occupied / free voxels
// Parameters "map.*" are declared on the owning node.
#pragma once

#include <chrono>
#include <future>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <rtabmap_msgs/msg/map_data.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "daedalus_exploration/exploration_map.hpp"

namespace daedalus_exploration
{

class MapManager
{
public:
  explicit MapManager(rclcpp::Node & node);

  const ExplorationMap & map() const {return *map_;}
  int rebuilds() const {return rebuilds_;}

private:
  // depth cloud kept for the rebuilds: points in the camera frame + camera pose in the odom frame
  struct StoredCloud
  {
    double stamp;                    // s
    Eigen::Isometry3d odom_cam;
    octomap::Pointcloud points;
  };
  // RTAB-Map keyframe: stamp + map -> odom correction given by its latest optimized pose
  struct Correction
  {
    double stamp;
    Eigen::Isometry3d map_odom;
  };

  rclcpp::Node & node_;
  std::unique_ptr<ExplorationMap> map_;

  std::string map_frame_, odom_frame_;
  int decimation_;                   // px: one depth pixel every `decimation` per row and column
  double rebuild_translation_;       // m, map -> odom change that triggers a rebuild
  double rebuild_rotation_;          // rad
  double store_translation_;         // m, camera motion between two stored clouds
  double store_rotation_;            // rad
  double store_voxel_;               // m, stored clouds thinned to one point per voxel of this size

  sensor_msgs::msg::Image::ConstSharedPtr depth_;      // latest depth image, inserted by the timer
  std::optional<sensor_msgs::msg::CameraInfo> info_;
  rclcpp::Time last_inserted_;                         // stamp of the last inserted image

  std::vector<std::shared_ptr<const StoredCloud>> clouds_;
  std::optional<Eigen::Isometry3d> last_stored_odom_cam_;
  size_t stored_points_ = 0;

  // RTAB-Map keyframes: id -> (stamp, odometry pose of base_link); id -> optimized pose (latest graph)
  std::map<int, std::pair<double, Eigen::Isometry3d>> keyframe_odom_;
  std::map<int, Eigen::Isometry3d> keyframe_map_;

  // map -> odom when the map was last built / rebuilt
  std::optional<Eigen::Isometry3d> reference_map_odom_;
  std::future<std::unique_ptr<ExplorationMap>> rebuild_;   // running rebuild
  size_t rebuild_clouds_ = 0;                              // clouds it uses (the first ones)
  std::chrono::steady_clock::time_point rebuild_start_;
  int rebuilds_ = 0;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<rtabmap_msgs::msg::MapData>::SharedPtr map_data_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr occupied_pub_, free_pub_;
  rclcpp::TimerBase::SharedPtr insert_timer_, rebuild_timer_, marker_timer_;

  void insert_depth();
  void store_cloud(double stamp, const Eigen::Isometry3d & odom_cam, const octomap::Pointcloud & cam_points);
  void on_map_data(const rtabmap_msgs::msg::MapData & msg);
  void check_rebuild();
  void finish_rebuild();
  void publish_markers();
};

}  // namespace daedalus_exploration
