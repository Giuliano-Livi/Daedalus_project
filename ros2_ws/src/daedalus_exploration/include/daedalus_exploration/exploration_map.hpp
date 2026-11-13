// 3D map for the exploration (AEP): OctoMap with unknown / free / occupied voxels, built from the depth
// camera and redrawn from the stored clouds after a large SLAM correction (see MapManager).
// No ROS here: the gain estimation and the planners only query this class.
#pragma once

#include <array>
#include <memory>
#include <optional>

#include <Eigen/Core>
#include <octomap/OcTree.h>

namespace daedalus_exploration
{

enum class VoxelState {unknown, free, occupied};

struct MapParams
{
  double resolution = 0.2;      // m, voxel side
  double prob_hit = 0.7;        // OctoMap sensor model (its defaults)
  double prob_miss = 0.4;
  double clamp_min = 0.12;
  double clamp_max = 0.97;
  double max_range = 5.0;       // m: farther depth points only clear the ray up to here
  // exploration volume V (map frame, z = 0 on the floor at the spawn point)
  double z_min = 0.0;
  double z_max = 5.0;
  std::optional<std::array<double, 4>> xy_box;   // x_min, x_max, y_min, y_max; none = unbounded
};

struct MapStats
{
  double free = 0.0;            // m^3 known free inside V
  double occupied = 0.0;        // m^3 known occupied inside V
};

class ExplorationMap
{
public:
  explicit ExplorationMap(const MapParams & params);

  // depth points in the map frame, measured from `origin` (camera centre): the voxels crossed by each
  // ray become more free, the end voxel more occupied; points beyond max_range only clear the ray
  void insert(const octomap::Pointcloud & points, const Eigen::Vector3d & origin);

  VoxelState state(const Eigen::Vector3d & p) const;
  bool in_bounds(const Eigen::Vector3d & p) const;
  MapStats stats() const;

  double resolution() const {return params_.resolution;}
  const MapParams & params() const {return params_;}
  const octomap::OcTree & tree() const {return *tree_;}

private:
  MapParams params_;
  std::unique_ptr<octomap::OcTree> tree_;

  std::unique_ptr<octomap::OcTree> make_tree() const;
  // volume of the cube (centre c, side s) inside V
  double volume_in_bounds(const octomap::point3d & c, double s) const;
};

}  // namespace daedalus_exploration
