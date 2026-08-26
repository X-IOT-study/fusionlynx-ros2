#pragma once

#include <string>
#include <string_view>

namespace state_fusion_backend
{

// Semantic names are intentionally independent from concrete GTSAM classes.
// For example, WheelBetween and LioBetween are both BetweenFactor<Pose3>.
enum class FactorType
{
  InitializationPrior,
  WheelBetween,
  LioBetween,
  LioPrior,
  StationaryAnchor,
  FallbackPrior,
  PlanarMotion,
  PlanarReference
};

constexpr std::string_view factor_type_name(FactorType type)
{
  switch (type) {
    case FactorType::InitializationPrior:
      return "initialization_prior";
    case FactorType::WheelBetween:
      return "wheel_between";
    case FactorType::LioBetween:
      return "lio_between";
    case FactorType::LioPrior:
      return "lio_prior";
    case FactorType::StationaryAnchor:
      return "stationary_anchor";
    case FactorType::FallbackPrior:
      return "fallback_prior";
    case FactorType::PlanarMotion:
      return "planar_motion";
    case FactorType::PlanarReference:
      return "planar_reference";
  }
  return "unknown";
}

enum class FactorReliabilityState
{
  Reliable,
  Degraded,
  Failed,
  Recovering
};

struct FactorReliabilityDecision
{
  FactorReliabilityState state{FactorReliabilityState::Reliable};
  double covariance_scale{1.0};
  bool accept_factor{true};
  std::string reason{"initial"};
};

constexpr std::string_view factor_reliability_state_name(
  FactorReliabilityState state)
{
  switch (state) {
    case FactorReliabilityState::Reliable:
      return "reliable";
    case FactorReliabilityState::Degraded:
      return "degraded";
    case FactorReliabilityState::Failed:
      return "failed";
    case FactorReliabilityState::Recovering:
      return "recovering";
  }
  return "unknown";
}

}  // namespace state_fusion_backend
