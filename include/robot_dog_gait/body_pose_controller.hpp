#pragma once

#include <array>

#include "robot_dog_gait/robot_dog_model.hpp"

namespace robot_dog_gait
{

struct BodyPose
{
  double roll{0.0};    // rad
  double pitch{0.0};   // rad
  double yaw{0.0};     // rad
  double height{0.0};  // m (base_link z above ground)
};

struct BodyPoseGains
{
  double kp_roll{1.0};
  double kp_pitch{1.0};
  double kp_height{1.0};
};

/// CoM / body pose offsets applied to stance feet (pitch, roll, height).
class BodyPoseController
{
public:
  explicit BodyPoseController(const RobotDogModel & model);

  void setGains(const BodyPoseGains & gains);
  void setTarget(const BodyPose & target);

  const BodyPose & target() const { return target_; }

  /// Return per-leg position offsets in the hip frame to realize body pose.
  std::array<Vec3, 4> footOffsets(
    const BodyPose & measured,
    const std::array<bool, 4> & in_stance) const;

private:
  RobotDogModel model_;
  BodyPoseGains gains_{};
  BodyPose target_{};
};

}  // namespace robot_dog_gait
