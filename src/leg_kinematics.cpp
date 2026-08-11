#include "robot_dog_gait/leg_kinematics.hpp"

#include <algorithm>
#include <cmath>

namespace robot_dog_gait
{

LegKinematics::LegKinematics(const LegGeometry & geom)
: geom_(geom)
{
}

Vec3 LegKinematics::forwardKinematics(const JointAngles & q) const
{
  // Hip frame: x forward, y left, z up. Chain: abad(x) → hip(y) → knee(y).
  const double l1 = geom_.l_abad;
  const double l2 = geom_.l_thigh;
  const double l3 = geom_.l_shank;
  const double side = geom_.side_sign;

  const double c1 = std::cos(q.hip_roll);
  const double s1 = std::sin(q.hip_roll);
  const double c2 = std::cos(q.hip_yaw);
  const double s2 = std::sin(q.hip_yaw);
  const double c23 = std::cos(q.hip_yaw + q.knee);
  const double s23 = std::sin(q.hip_yaw + q.knee);

  const double leg_xz = l2 * c2 + l3 * c23;

  Vec3 p;
  p.x = l2 * s2 + l3 * s23;
  p.y = side * (l1 * c1 + leg_xz * s1);
  p.z = -side * l1 * s1 - leg_xz * c1;
  return p;
}

bool LegKinematics::inverseKinematics(const Vec3 & foot_hip, JointAngles & q_out) const
{
  const double l1 = geom_.l_abad;
  const double l2 = geom_.l_thigh;
  const double l3 = geom_.l_shank;
  const double side = geom_.side_sign;

  const double x = foot_hip.x;
  const double y = foot_hip.y;
  const double z = foot_hip.z;

  const double yz2 = y * y + z * z;
  if (yz2 < l1 * l1) {
    return false;
  }

  const double r = std::sqrt(yz2 - l1 * l1);
  const double hip_roll =
    std::atan2(y, -z) - std::atan2(side * l1, r);

  const double xz_len = std::sqrt(x * x + r * r);
  if (xz_len < 1e-6 || xz_len > (l2 + l3) || xz_len < std::abs(l2 - l3)) {
    return false;
  }

  const double cos_knee = (l2 * l2 + l3 * l3 - xz_len * xz_len) / (2.0 * l2 * l3);
  const double knee = -std::acos(std::clamp(cos_knee, -1.0, 1.0));

  const double cos_thigh = (l2 * l2 + xz_len * xz_len - l3 * l3) / (2.0 * l2 * xz_len);
  const double hip_yaw = std::atan2(-x, r) - std::acos(std::clamp(cos_thigh, -1.0, 1.0));

  q_out.hip_roll = hip_roll;
  q_out.hip_yaw = hip_yaw;
  q_out.knee = knee;
  return std::isfinite(hip_roll) && std::isfinite(hip_yaw) && std::isfinite(knee);
}

}  // namespace robot_dog_gait
