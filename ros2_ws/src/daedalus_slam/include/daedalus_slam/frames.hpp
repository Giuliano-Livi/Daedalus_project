// Frame conventions shared by px4_odometry and vision_to_px4.
#pragma once

#include <cmath>

#include <Eigen/Dense>

namespace daedalus_slam
{

// NED -> ENU (swap x/y, flip z) and FRD -> FLU (flip y, z); both are their own inverse
inline const Eigen::Matrix3d & ned_to_enu()
{
  static const Eigen::Matrix3d m = (Eigen::Matrix3d() << 0, 1, 0, 1, 0, 0, 0, 0, -1).finished();
  return m;
}

inline const Eigen::Matrix3d & frd_to_flu()
{
  static const Eigen::Matrix3d m = Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal();
  return m;
}

// rotation matrix of the quaternion (w, x, y, z)
inline Eigen::Matrix3d quat_to_rot(double w, double x, double y, double z)
{
  Eigen::Matrix3d r;
  r << 1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
    2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
    2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y);
  return r;
}

// quaternion (w, x, y, z) of a rotation matrix; branch on the largest diagonal term for numerical
// stability
inline Eigen::Vector4d rot_to_quat(const Eigen::Matrix3d & r)
{
  const double t = r.trace();
  if (t > 0) {
    const double s = 2 * std::sqrt(t + 1);
    return {0.25 * s, (r(2, 1) - r(1, 2)) / s, (r(0, 2) - r(2, 0)) / s, (r(1, 0) - r(0, 1)) / s};
  }
  int i = 0;
  if (r(1, 1) > r(i, i)) {i = 1;}
  if (r(2, 2) > r(i, i)) {i = 2;}
  const int j = (i + 1) % 3, k = (i + 2) % 3;
  const double s = 2 * std::sqrt(1 + r(i, i) - r(j, j) - r(k, k));
  double q[3];
  q[i] = 0.25 * s;
  q[j] = (r(j, i) + r(i, j)) / s;
  q[k] = (r(k, i) + r(i, k)) / s;
  return {(r(k, j) - r(j, k)) / s, q[0], q[1], q[2]};
}

}  // namespace daedalus_slam
