#pragma once

#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "state_fusion_backend/factor_types.hpp"
#include "state_fusion_backend/fusion_backend.hpp"

namespace state_fusion_backend
{

struct FactorBuildContext
{
  gtsam::Key previous_pose_key{0};
  gtsam::Key pose_key{0};
  double stamp{0.0};
  gtsam::Pose3 previous_pose;
  gtsam::Pose3 initial_pose;

  std::optional<gtsam::Pose3> wheel_delta;
  std::optional<gtsam::Pose3> lio_delta;
  std::optional<LioObservation> lio;
  std::optional<LioObservation> previous_lio;
  gtsam::Pose3 aligned_lio;
  double lio_scale{1.0};
  FactorReliabilityDecision lio_reliability;

  bool use_lio{false};
  bool stationary{false};
  std::optional<gtsam::Pose3> stationary_anchor;
};

struct FactorBuildResult
{
  gtsam::NonlinearFactorGraph factors;
  gtsam::Values values;
  gtsam::FixedLagSmoother::KeyTimestampMap timestamps;
  std::vector<FactorType> factor_types;
  struct FactorDecision
  {
    FactorType type;
    bool accepted{false};
    double covariance_scale{1.0};
    std::string reason;
  };
  std::vector<FactorDecision> decisions;
  bool used_wheel{false};
  bool used_lio{false};

  void record_decision(
    FactorType type,
    bool accepted,
    double covariance_scale,
    std::string reason)
  {
    decisions.push_back(
      FactorDecision{type, accepted, covariance_scale, std::move(reason)});
    if (accepted) {
      factor_types.push_back(type);
    }
  }
};

class FactorGraphBuilder final
{
public:
  explicit FactorGraphBuilder(const FusionParameters & parameters);

  FactorBuildResult build_initialization(
    gtsam::Key pose_key,
    const gtsam::Pose3 & initial_pose,
    double stamp) const;

  FactorBuildResult build_motion(const FactorBuildContext & context) const;

  std::vector<FactorType> registered_factor_types() const;

private:
  using MotionProvider =
    std::function<void(const FactorBuildContext &, FactorBuildResult &)>;

  struct ProviderRegistration
  {
    FactorType type;
    MotionProvider build;
  };

  void register_default_providers();

  gtsam::SharedNoiseModel robust_noise(
    const gtsam::SharedNoiseModel & gaussian) const;
  gtsam::SharedNoiseModel make_initial_pose_noise() const;
  gtsam::SharedNoiseModel make_lio_noise(
    const LioObservation & lio,
    const std::optional<LioObservation> & previous_lio,
    double scale) const;
  gtsam::SharedNoiseModel make_stationary_pose_noise() const;
  gtsam::SharedNoiseModel make_wheel_noise() const;
  gtsam::SharedNoiseModel make_planar_motion_noise() const;
  gtsam::SharedNoiseModel make_planar_reference_noise() const;

  FusionParameters parameters_;
  std::vector<ProviderRegistration> providers_;
};

}  // namespace state_fusion_backend
