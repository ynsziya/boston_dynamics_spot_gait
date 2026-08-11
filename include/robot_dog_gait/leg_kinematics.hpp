#pragma once

#include "robot_dog_gait/robot_dog_model.hpp"

namespace robot_dog_gait
{

struct JointAngles
{
  double hip_roll{0.0};   // abad / abduction
  double hip_yaw{0.0};    // hip pitch (thigh)
  double knee{0.0};       // knee
};

/// Single-leg analytic FK/IK, parameterized by LegGeometry (works for all 4 legs).
class LegKinematics
{
public:
  explicit LegKinematics(const LegGeometry & geom);

  /// Foot position in the hip frame [m].
  Vec3 forwardKinematics(const JointAngles & q) const;

  /// Inverse kinematics. Returns false if target is unreachable.
  bool inverseKinematics(const Vec3 & foot_hip, JointAngles & q_out) const;

  const LegGeometry & geometry() const { return geom_; }

private:
  LegGeometry geom_;
};

}  // namespace robot_dog_gait
