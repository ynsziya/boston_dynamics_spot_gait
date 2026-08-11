#include "robot_dog_gait/gait_engine.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace robot_dog_gait
{

GaitEngine::GaitEngine(const GaitParams & params)
: params_(params)
{
}

void GaitEngine::setParams(const GaitParams & params)
{
  params_ = params;
}

void GaitEngine::setGait(GaitType type)
{
  params_.type = type;
}

void GaitEngine::reset(double t0)
{
  time_ = t0;
}

std::array<double, 4> GaitEngine::phaseOffsets(GaitType type)
{
  // Order: FL, FR, RL, RR
  switch (type) {
    case GaitType::Trot:
      return {0.0, 0.5, 0.5, 0.0};       // diagonal pairs
    case GaitType::Walk:
      return {0.0, 0.5, 0.75, 0.25};     // lateral sequence
    case GaitType::Pace:
      return {0.0, 0.5, 0.0, 0.5};       // ipsilateral pairs
    case GaitType::Bound:
      return {0.0, 0.0, 0.5, 0.5};       // front/hind pairs
    default:
      return {0.0, 0.5, 0.5, 0.0};
  }
}

GaitType GaitEngine::gaitFromString(const std::string & name)
{
  std::string n = name;
  std::transform(n.begin(), n.end(), n.begin(), ::tolower);
  if (n == "walk") {return GaitType::Walk;}
  if (n == "trot") {return GaitType::Trot;}
  if (n == "pace") {return GaitType::Pace;}
  if (n == "bound") {return GaitType::Bound;}
  throw std::invalid_argument("Unknown gait: " + name);
}

const char * GaitEngine::toString(GaitType type)
{
  switch (type) {
    case GaitType::Walk: return "walk";
    case GaitType::Trot: return "trot";
    case GaitType::Pace: return "pace";
    case GaitType::Bound: return "bound";
    default: return "trot";
  }
}

std::array<LegPhase, 4> GaitEngine::update(double dt)
{
  time_ += dt;
  const double freq = std::max(1e-3, params_.frequency);
  const double duty = std::clamp(params_.duty_factor, 0.05, 0.95);
  const double swing_frac = 1.0 - duty;
  const auto offsets = phaseOffsets(params_.type);

  std::array<LegPhase, 4> out{};
  for (int i = 0; i < 4; ++i) {
    double ph = std::fmod(time_ * freq + offsets[static_cast<size_t>(i)], 1.0);
    if (ph < 0.0) {
      ph += 1.0;
    }
    LegPhase lp;
    lp.phase = ph;
    // Phase 0..swing_frac = swing, rest = stance (common convention).
    if (ph < swing_frac) {
      lp.in_swing = true;
      lp.swing_progress = (swing_frac > 1e-9) ? (ph / swing_frac) : 0.0;
      lp.stance_progress = 0.0;
    } else {
      lp.in_swing = false;
      lp.swing_progress = 1.0;
      lp.stance_progress = (duty > 1e-9) ? ((ph - swing_frac) / duty) : 0.0;
    }
    out[static_cast<size_t>(i)] = lp;
  }
  return out;
}

}  // namespace robot_dog_gait
