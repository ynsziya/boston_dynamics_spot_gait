#include "robot_dog_gait/body_pose_controller.hpp"

#include <cmath>

namespace robot_dog_gait
{

BodyPoseController::BodyPoseController(const RobotDogModel & model)
: model_(model)
{
  target_.height = model_.nominal_height;
}

void BodyPoseController::setGains(const BodyPoseGains & gains)
{
  gains_ = gains;
}

void BodyPoseController::setTarget(const BodyPose & target)
{
  target_ = target;
}

std::array<Vec3, 4> BodyPoseController::footOffsets(
  const BodyPose & measured,
  const std::array<bool, 4> & in_stance) const
{
  const double roll_err = gains_.kp_roll * (target_.roll - measured.roll);
  const double pitch_err = gains_.kp_pitch * (target_.pitch - measured.pitch);
  const double height_err = gains_.kp_height * (target_.height - measured.height);

  std::array<Vec3, 4> offsets{};
  for (int i = 0; i < 4; ++i) {
    if (!in_stance[static_cast<size_t>(i)]) {
      continue;
    }
    const auto & hip = model_.legs[static_cast<size_t>(i)].hip_offset;
    // Small-angle: Δz ≈ -pitch * x + roll * y + height
    offsets[static_cast<size_t>(i)].x = 0.0;
    offsets[static_cast<size_t>(i)].y = 0.0;
    offsets[static_cast<size_t>(i)].z =
      -pitch_err * hip.x + roll_err * hip.y - height_err;
  }
  return offsets;
}

}  // namespace robot_dog_gait
