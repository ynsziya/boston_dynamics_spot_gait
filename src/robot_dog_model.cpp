#include "robot_dog_gait/robot_dog_model.hpp"

#include <stdexcept>

namespace robot_dog_gait
{

namespace
{
LegGeometry makeLeg(const Vec3 & hip, double side_sign, double l_abad, double l_thigh, double l_shank)
{
  LegGeometry g;
  g.hip_offset = hip;
  g.side_sign = side_sign;
  g.l_abad = l_abad;
  g.l_thigh = l_thigh;
  g.l_shank = l_shank;
  return g;
}
}  // namespace

RobotDogModel RobotDogModel::fromDefaults()
{
  // Values derived from bosdyn_spot_ros2/urdf/spot_zero.urdf (spot_zero).
  constexpr double l_abad = 0.0860;   // approx |hip_yaw origin| in yz
  constexpr double l_thigh = 0.4050;  // |knee origin from hip_yaw|
  constexpr double l_shank = 0.3800;  // approx hip-height - thigh projection

  RobotDogModel m;
  m.body_length = 0.5600;   // |fl_x - rl_x|
  m.body_width = 0.1000;    // approx left-right hip spacing
  m.nominal_height = 0.55;

  m.legs[static_cast<int>(LegId::FL)] =
    makeLeg({0.335676, 0.085451, 0.086617}, +1.0, l_abad, l_thigh, l_shank);
  m.legs[static_cast<int>(LegId::FR)] =
    makeLeg({0.335676, -0.014549, 0.086617}, -1.0, l_abad, l_thigh, l_shank);
  m.legs[static_cast<int>(LegId::RL)] =
    makeLeg({-0.224324, 0.085451, 0.086617}, +1.0, l_abad, l_thigh, l_shank);
  m.legs[static_cast<int>(LegId::RR)] =
    makeLeg({-0.224324, -0.014549, 0.086617}, -1.0, l_abad, l_thigh, l_shank);
  return m;
}

const LegGeometry & RobotDogModel::leg(LegId id) const
{
  const int i = static_cast<int>(id);
  if (i < 0 || i >= static_cast<int>(LegId::COUNT)) {
    throw std::out_of_range("LegId out of range");
  }
  return legs[static_cast<size_t>(i)];
}

LegGeometry & RobotDogModel::leg(LegId id)
{
  return const_cast<LegGeometry &>(static_cast<const RobotDogModel *>(this)->leg(id));
}

}  // namespace robot_dog_gait
