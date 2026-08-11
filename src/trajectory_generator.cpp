#include "robot_dog_gait/trajectory_generator.hpp"

#include <algorithm>
#include <cmath>

namespace robot_dog_gait
{

namespace
{
Vec3 operator+(const Vec3 & a, const Vec3 & b) {return {a.x + b.x, a.y + b.y, a.z + b.z};}
Vec3 operator-(const Vec3 & a, const Vec3 & b) {return {a.x - b.x, a.y - b.y, a.z - b.z};}
Vec3 operator*(const Vec3 & a, double s) {return {a.x * s, a.y * s, a.z * s};}
}  // namespace

TrajectoryGenerator::TrajectoryGenerator(const RobotDogModel & model, const GaitParams & gait)
: model_(model), gait_(gait)
{
  // Default nominal feet: under hip, at -nominal_height in hip frame (approx).
  for (int i = 0; i < 4; ++i) {
    nominal_feet_[static_cast<size_t>(i)] = {0.0, 0.0, -model_.nominal_height};
  }
}

void TrajectoryGenerator::setGaitParams(const GaitParams & gait)
{
  gait_ = gait;
}

void TrajectoryGenerator::setNominalFoot(LegId id, const Vec3 & foot_hip)
{
  nominal_feet_[static_cast<size_t>(id)] = foot_hip;
}

Vec3 TrajectoryGenerator::cubicBezier(
  const Vec3 & p0, const Vec3 & p1, const Vec3 & p2, const Vec3 & p3, double s)
{
  const double u = 1.0 - s;
  return p0 * (u * u * u) + p1 * (3.0 * u * u * s) + p2 * (3.0 * u * s * s) + p3 * (s * s * s);
}

Vec3 TrajectoryGenerator::cubicBezierDerivative(
  const Vec3 & p0, const Vec3 & p1, const Vec3 & p2, const Vec3 & p3, double s)
{
  const double u = 1.0 - s;
  return (p1 - p0) * (3.0 * u * u) + (p2 - p1) * (6.0 * u * s) + (p3 - p2) * (3.0 * s * s);
}

FootTrajectory TrajectoryGenerator::compute(
  LegId id,
  const LegPhase & phase,
  const Twist2D & cmd) const
{
  const auto & hip = model_.leg(id).hip_offset;
  const Vec3 nom = nominal_feet_[static_cast<size_t>(id)];

  // Raibert-style foothold relative to nominal, expressed in hip frame.
  const double stance_T = (gait_.frequency > 1e-6) ?
    (gait_.duty_factor / gait_.frequency) : 0.0;
  Vec3 step{
    cmd.vx * stance_T * 0.5 + hip.y * cmd.wz * stance_T * 0.5,
    cmd.vy * stance_T * 0.5 - hip.x * cmd.wz * stance_T * 0.5,
    0.0};

  // Clamp step using configured lengths.
  const double sx = std::max(1e-6, gait_.step_length);
  const double sy = std::max(1e-6, gait_.step_width);
  step.x = std::clamp(step.x, -sx, sx);
  step.y = std::clamp(step.y, -sy, sy);

  const Vec3 p_start = nom + step * (-1.0);
  const Vec3 p_end = nom + step;

  FootTrajectory out;
  if (phase.in_swing) {
    const double s = std::clamp(phase.swing_progress, 0.0, 1.0);
    const Vec3 p1{p_start.x, p_start.y, p_start.z + gait_.swing_height};
    const Vec3 p2{p_end.x, p_end.y, p_end.z + gait_.swing_height};
    out.position = cubicBezier(p_start, p1, p2, p_end, s);
    // Velocity roughly scaled by swing duration.
    const double swing_T = (gait_.frequency > 1e-6) ?
      ((1.0 - gait_.duty_factor) / gait_.frequency) : 1.0;
    const Vec3 d = cubicBezierDerivative(p_start, p1, p2, p_end, s);
    out.velocity = d * ((swing_T > 1e-6) ? (1.0 / swing_T) : 0.0);
  } else {
    const double s = std::clamp(phase.stance_progress, 0.0, 1.0);
    out.position = p_end + (p_start - p_end) * s;
    out.position.z = nom.z + gait_.stance_depth;
    const double stance_T_safe = std::max(1e-6, stance_T);
    out.velocity = (p_start - p_end) * (1.0 / stance_T_safe);
  }
  return out;
}

}  // namespace robot_dog_gait
