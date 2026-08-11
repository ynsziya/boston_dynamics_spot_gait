#pragma once

#include <array>
#include <string>

#include "robot_dog_gait/robot_dog_model.hpp"

namespace robot_dog_gait
{

enum class GaitType
{
  Walk,
  Trot,
  Pace,
  Bound
};

struct GaitParams
{
  GaitType type{GaitType::Trot};
  double frequency{1.5};       // Hz (full cycle)
  double duty_factor{0.5};     // stance fraction of cycle [0,1]
  double swing_height{0.06};   // m
  double stance_depth{0.0};    // m (negative = dig into ground)
  double step_length{0.12};    // m at unit forward command
  double step_width{0.06};     // m at unit lateral command
};

struct LegPhase
{
  double phase{0.0};           // [0,1) normalized cycle phase
  bool in_swing{false};
  double swing_progress{0.0};  // [0,1] within swing window
  double stance_progress{0.0}; // [0,1] within stance window
};

/// Phase generator for common quadruped gaits (trot, walk, pace, bound).
class GaitEngine
{
public:
  explicit GaitEngine(const GaitParams & params = {});

  void setParams(const GaitParams & params);
  const GaitParams & params() const { return params_; }

  void setGait(GaitType type);
  void reset(double t0 = 0.0);

  /// Advance internal clock and return per-leg phases.
  std::array<LegPhase, 4> update(double dt);

  /// Relative phase offsets for current gait (FL, FR, RL, RR).
  static std::array<double, 4> phaseOffsets(GaitType type);

  static GaitType gaitFromString(const std::string & name);
  static const char * toString(GaitType type);

private:
  GaitParams params_;
  double time_{0.0};
};

}  // namespace robot_dog_gait
