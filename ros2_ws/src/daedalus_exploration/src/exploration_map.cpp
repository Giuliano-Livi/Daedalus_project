#include "daedalus_exploration/exploration_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace daedalus_exploration
{

ExplorationMap::ExplorationMap(const MapParams & params)
: params_(params), tree_(make_tree())
{
}

std::unique_ptr<octomap::OcTree> ExplorationMap::make_tree() const
{
  auto tree = std::make_unique<octomap::OcTree>(params_.resolution);
  tree->setProbHit(params_.prob_hit);
  tree->setProbMiss(params_.prob_miss);
  tree->setClampingThresMin(params_.clamp_min);
  tree->setClampingThresMax(params_.clamp_max);
  return tree;
}

void ExplorationMap::insert(const octomap::Pointcloud & points, const Eigen::Vector3d & origin)
{
  // discretize: rays to points in the same voxel are traced once (much faster, same result at our
  // resolution); lazy_eval false keeps the inner nodes up to date for the queries
  tree_->insertPointCloud(points, octomap::point3d(origin.x(), origin.y(), origin.z()),
    params_.max_range, false, true);
}

VoxelState ExplorationMap::state(const Eigen::Vector3d & p) const
{
  const octomap::OcTreeNode * node = tree_->search(p.x(), p.y(), p.z());
  if (!node) {return VoxelState::unknown;}
  return tree_->isNodeOccupied(node) ? VoxelState::occupied : VoxelState::free;
}

bool ExplorationMap::in_bounds(const Eigen::Vector3d & p) const
{
  if (p.z() < params_.z_min || p.z() > params_.z_max) {return false;}
  if (!params_.xy_box) {return true;}
  const auto & b = *params_.xy_box;
  return p.x() >= b[0] && p.x() <= b[1] && p.y() >= b[2] && p.y() <= b[3];
}

double ExplorationMap::volume_in_bounds(const octomap::point3d & c, double s) const
{
  constexpr double inf = std::numeric_limits<double>::infinity();
  const auto & b = params_.xy_box;
  const double lo[3] = {b ? (*b)[0] : -inf, b ? (*b)[2] : -inf, params_.z_min};
  const double hi[3] = {b ? (*b)[1] : inf, b ? (*b)[3] : inf, params_.z_max};
  double volume = 1.0;
  for (int a = 0; a < 3; ++a) {
    const double overlap = std::min(c(a) + s / 2, hi[a]) - std::max(c(a) - s / 2, lo[a]);
    if (overlap <= 0) {return 0.0;}
    volume *= overlap;
  }
  return volume;
}

MapStats ExplorationMap::stats() const
{
  MapStats stats;
  for (auto it = tree_->begin_leafs(), end = tree_->end_leafs(); it != end; ++it) {
    const double v = volume_in_bounds(it.getCoordinate(), it.getSize());
    (tree_->isNodeOccupied(*it) ? stats.occupied : stats.free) += v;
  }
  return stats;
}

}  // namespace daedalus_exploration
