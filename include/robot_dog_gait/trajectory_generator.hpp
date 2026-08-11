#pragma once

#include "robot_dog_gait/gait_engine.hpp"
#include "robot_dog_gait/robot_dog_model.hpp"

namespace robot_dog_gait
{

struct Twist2D
{
  double vx{0.0};  // m/s body x
  double vy{0.0};  // m/s body y
  double wz{0.0};  // rad/s yaw
};

struct FootTrajectory
{
  Vec3 position{};   // hip frame [m]
  Vec3 velocity{};   // hip frame [m/s]
};

/// Swing (cubic Bezier) + stance (linear) foot trajectories in the hip frame.
class TrajectoryGenerator
{
public:
  TrajectoryGenerator(const RobotDogModel & model, const GaitParams & gait);

  void setGaitParams(const GaitParams & gait);
  void setNominalFoot(LegId id, const Vec3 & foot_hip);

  /// Compute foot target for one leg given phase and body velocity command.
  FootTrajectory compute(
    LegId id,
    const LegPhase & phase,
    const Twist2D & cmd) const;

private:
  static Vec3 cubicBezier(const Vec3 & p0, const Vec3 & p1, const Vec3 & p2, const Vec3 & p3, double s);
  static Vec3 cubicBezierDerivative(
    const Vec3 & p0, const Vec3 & p1, const Vec3 & p2, const Vec3 & p3, double s);

  RobotDogModel model_;
  GaitParams gait_;
  std::array<Vec3, 4> nominal_feet_{};
};

}  // namespace robot_dog_gait
