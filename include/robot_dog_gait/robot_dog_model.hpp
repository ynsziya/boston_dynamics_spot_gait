#pragma once

#include <array>

namespace robot_dog_gait
{

enum class LegId : int
{
  FL = 0,
  FR = 1,
  RL = 2,
  RR = 3,
  COUNT = 4
};

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct LegGeometry
{
  /// Hip roll joint origin in base_link frame [m].
  Vec3 hip_offset{};
  /// Link lengths: abad (lateral), thigh, shank [m].
  double l_abad{0.0};
  double l_thigh{0.0};
  double l_shank{0.0};
  /// +1 left, -1 right — mirrors IK/FK lateral sign.
  double side_sign{1.0};
};

struct RobotDogModel
{
  double body_length{0.0};
  double body_width{0.0};
  double nominal_height{0.0};
  std::array<LegGeometry, 4> legs{};

  /// Load from ROS parameters / YAML (robot_dimensions.yaml).
  static RobotDogModel fromDefaults();

  const LegGeometry & leg(LegId id) const;
  LegGeometry & leg(LegId id);
};

inline const char * toString(LegId id)
{
  switch (id) {
    case LegId::FL: return "FL";
    case LegId::FR: return "FR";
    case LegId::RL: return "RL";
    case LegId::RR: return "RR";
    default: return "?";
  }
}

}  // namespace robot_dog_gait
