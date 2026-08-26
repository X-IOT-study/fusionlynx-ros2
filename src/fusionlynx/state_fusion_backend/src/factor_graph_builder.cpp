#include "state_fusion_backend/factor_graph_builder.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include "state_fusion_backend/factors.hpp"

namespace state_fusion_backend
{
namespace
{

gtsam::SharedNoiseModel diagonal_noise(const gtsam::Vector & sigmas)
{
  return gtsam::noiseModel::Diagonal::Sigmas(sigmas);
}

gtsam::SharedNoiseModel robustify(
  const gtsam::SharedNoiseModel & gaussian,
  double scale)
{
  return gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Huber::Create(scale),
    gaussian);
}

template<typename FactorT, typename... Args>
void add_factor(
  FactorBuildResult & result,
  FactorType type,
  Args && ... args)
{
  result.factors.add(
    boost::make_shared<FactorT>(std::forward<Args>(args)...));
  (void)type;
}

}  // namespace

FactorGraphBuilder::FactorGraphBuilder(const FusionParameters & parameters)
: parameters_(parameters)
{
  register_default_providers();
}

gtsam::SharedNoiseModel FactorGraphBuilder::robust_noise(
  const gtsam::SharedNoiseModel & gaussian) const
{
  if (parameters_.robust_kernel == "none" ||
    parameters_.robust_kernel == "gaussian")
  {
    return gaussian;
  }
  return robustify(gaussian, parameters_.robust_kernel_scale);
}

gtsam::SharedNoiseModel FactorGraphBuilder::make_initial_pose_noise() const
{
  return diagonal_noise(
    gtsam::Vector6::Constant(parameters_.initial_pose_sigma));
}

gtsam::SharedNoiseModel FactorGraphBuilder::make_lio_noise(
  const LioObservation & lio,
  const std::optional<LioObservation> & previous_lio,
  double scale) const
{
  gtsam::Vector6 sigmas;
  // A relative LIO factor is formed from two absolute odometry samples.
  // Approximate the increment covariance by the sum of the independent
  // endpoint covariances. This keeps the covariance policy local to LIO.
  constexpr int ros_index_for_gtsam[] = {3, 4, 5, 0, 1, 2};
  for (int i = 0; i < 6; ++i) {
    const double fallback_sigma = i < 3 ?
      parameters_.lio_rotation_sigma : parameters_.lio_position_sigma;
    const double current_variance =
      lio.covariance_valid ?
      lio.covariance(ros_index_for_gtsam[i], ros_index_for_gtsam[i]) :
      std::numeric_limits<double>::quiet_NaN();
    const double previous_variance =
      previous_lio && previous_lio->covariance_valid ?
      previous_lio->covariance(
      ros_index_for_gtsam[i], ros_index_for_gtsam[i]) :
      std::numeric_limits<double>::quiet_NaN();
    const bool current_valid =
      std::isfinite(current_variance) &&
      current_variance > 1.0e-12 &&
      current_variance <= 1.0e6;
    const bool previous_valid =
      std::isfinite(previous_variance) &&
      previous_variance > 1.0e-12 &&
      previous_variance <= 1.0e6;
    double variance = 0.0;
    if (current_valid) {
      variance += current_variance;
    }
    if (previous_valid) {
      variance += previous_variance;
    }
    // FAST-LIO covariance can be overconfident for the external fused
    // trajectory. Keep the configured system noise as a floor.
    const double floor_sigma = i < 3 ?
      parameters_.lio_rotation_sigma : parameters_.lio_position_sigma;
    variance = std::max(variance, floor_sigma * floor_sigma);
    if (variance <= 1.0e-12) {
      variance = fallback_sigma * fallback_sigma;
    }
    sigmas(i) = std::sqrt(variance);
    if (i == 5) {
      sigmas(i) *= std::max(parameters_.lio_z_sigma_scale, 1.0);
    }
  }
  sigmas *= std::max(scale * parameters_.lio_noise_scale, 1.0e-6);
  return robust_noise(diagonal_noise(sigmas));
}

gtsam::SharedNoiseModel FactorGraphBuilder::make_stationary_pose_noise() const
{
  return diagonal_noise(
    gtsam::Vector6::Constant(parameters_.stationary_pose_sigma));
}

gtsam::SharedNoiseModel FactorGraphBuilder::make_wheel_noise() const
{
  gtsam::Vector6 sigmas;
  // Wheel odometry constrains planar translation and yaw only.
  sigmas << 1000.0,
    1000.0,
    parameters_.wheel_yaw_sigma,
    parameters_.wheel_x_sigma,
    parameters_.wheel_y_sigma,
    1000.0;
  return robust_noise(diagonal_noise(sigmas));
}

gtsam::SharedNoiseModel FactorGraphBuilder::make_planar_motion_noise() const
{
  gtsam::Vector6 sigmas;
  // Relative soft prior for roll/pitch stability and zero vertical motion.
  sigmas << parameters_.planar_roll_sigma,
    parameters_.planar_pitch_sigma,
    1000.0,
    1000.0,
    1000.0,
    parameters_.planar_z_sigma;
  return diagonal_noise(sigmas);
}

gtsam::SharedNoiseModel FactorGraphBuilder::make_planar_reference_noise() const
{
  gtsam::Vector3 sigmas;
  sigmas << parameters_.planar_z_sigma,
    parameters_.planar_roll_sigma,
    parameters_.planar_pitch_sigma;
  return diagonal_noise(sigmas);
}

FactorBuildResult FactorGraphBuilder::build_initialization(
  gtsam::Key pose_key,
  const gtsam::Pose3 & initial_pose,
  double stamp) const
{
  FactorBuildResult result;
  add_factor<gtsam::PriorFactor<gtsam::Pose3>>(
    result,
    FactorType::InitializationPrior,
    pose_key,
    initial_pose,
    make_initial_pose_noise());
  result.record_decision(
    FactorType::InitializationPrior, true, 1.0, "initialization_prior_added");
  result.values.insert(pose_key, initial_pose);
  result.timestamps[pose_key] = stamp;
  return result;
}

FactorBuildResult FactorGraphBuilder::build_motion(
  const FactorBuildContext & context) const
{
  FactorBuildResult result;
  result.values.insert(context.pose_key, context.initial_pose);
  result.timestamps[context.pose_key] = context.stamp;

  for (const auto & provider : providers_) {
    provider.build(context, result);
  }

  return result;
}

std::vector<FactorType> FactorGraphBuilder::registered_factor_types() const
{
  std::vector<FactorType> types;
  types.reserve(providers_.size());
  for (const auto & provider : providers_) {
    types.push_back(provider.type);
  }
  return types;
}

void FactorGraphBuilder::register_default_providers()
{
  providers_.push_back(
    ProviderRegistration{
      FactorType::WheelBetween,
      [this](const FactorBuildContext & context, FactorBuildResult & result) {
        if (!context.wheel_delta) {
          result.record_decision(
            FactorType::WheelBetween, false, 1.0, "wheel_observation_unavailable");
          return;
        }
        add_factor<gtsam::BetweenFactor<gtsam::Pose3>>(
          result,
          FactorType::WheelBetween,
          context.previous_pose_key,
          context.pose_key,
          *context.wheel_delta,
          make_wheel_noise());
        result.record_decision(
          FactorType::WheelBetween, true, 1.0, "wheel_between_added");
        result.used_wheel = true;
      }});

  providers_.push_back(
    ProviderRegistration{
      FactorType::LioBetween,
      [this](const FactorBuildContext & context, FactorBuildResult & result) {
        if (!context.use_lio || !context.lio || !context.lio_delta) {
          result.record_decision(
            FactorType::LioBetween, false, context.lio_scale,
            context.lio ? "relative_increment_unavailable" :
            "lio_observation_unavailable");
          return;
        }
        if (!context.lio_reliability.accept_factor) {
          result.record_decision(
            FactorType::LioBetween, false, context.lio_scale,
            context.lio_reliability.reason);
          return;
        }
        add_factor<gtsam::BetweenFactor<gtsam::Pose3>>(
          result,
          FactorType::LioBetween,
          context.previous_pose_key,
          context.pose_key,
          *context.lio_delta,
          make_lio_noise(*context.lio, context.previous_lio, context.lio_scale));
        result.record_decision(
          FactorType::LioBetween, true, context.lio_scale,
          context.lio_reliability.reason);
        result.used_lio = true;
      }});

  providers_.push_back(
    ProviderRegistration{
      FactorType::LioPrior,
      [this](const FactorBuildContext & context, FactorBuildResult & result) {
        if (!context.use_lio || !context.lio || context.lio_delta) {
          result.record_decision(
            FactorType::LioPrior, false, context.lio_scale,
            context.lio ? "relative_increment_available" :
            "lio_observation_unavailable");
          return;
        }
        if (!context.lio_reliability.accept_factor) {
          result.record_decision(
            FactorType::LioPrior, false, context.lio_scale,
            context.lio_reliability.reason);
          return;
        }
        add_factor<gtsam::PriorFactor<gtsam::Pose3>>(
          result,
          FactorType::LioPrior,
          context.pose_key,
          context.aligned_lio,
          make_lio_noise(*context.lio, context.previous_lio, context.lio_scale));
        result.record_decision(
          FactorType::LioPrior, true, context.lio_scale,
          context.lio_reliability.reason);
        result.used_lio = true;
      }});

  providers_.push_back(
    ProviderRegistration{
      FactorType::StationaryAnchor,
      [this](const FactorBuildContext & context, FactorBuildResult & result) {
        if (!context.stationary || !context.stationary_anchor) {
          result.record_decision(
            FactorType::StationaryAnchor, false, 1.0,
            "stationary_anchor_unavailable");
          return;
        }
        add_factor<gtsam::PriorFactor<gtsam::Pose3>>(
          result,
          FactorType::StationaryAnchor,
          context.pose_key,
          *context.stationary_anchor,
          make_stationary_pose_noise());
        result.record_decision(
          FactorType::StationaryAnchor, true, 1.0, "stationary_anchor_added");
      }});

  providers_.push_back(
    ProviderRegistration{
      FactorType::FallbackPrior,
      [this](const FactorBuildContext & context, FactorBuildResult & result) {
        if (result.used_lio || result.used_wheel) {
          result.record_decision(
            FactorType::FallbackPrior, false, 1.0,
            "primary_motion_factor_available");
          return;
        }
        add_factor<gtsam::PriorFactor<gtsam::Pose3>>(
          result,
          FactorType::FallbackPrior,
          context.pose_key,
          context.previous_pose,
          diagonal_noise(gtsam::Vector6::Constant(parameters_.fallback_pose_sigma)));
        result.record_decision(
          FactorType::FallbackPrior, true, 1.0, "fallback_prior_added");
      }});

  providers_.push_back(
    ProviderRegistration{
      FactorType::PlanarMotion,
      [this](const FactorBuildContext & context, FactorBuildResult & result) {
        if (!parameters_.use_planar_prior) {
          result.record_decision(
            FactorType::PlanarMotion, false, 1.0, "planar_prior_disabled");
          return;
        }
        add_factor<gtsam::BetweenFactor<gtsam::Pose3>>(
          result,
          FactorType::PlanarMotion,
          context.previous_pose_key,
          context.pose_key,
          gtsam::Pose3(),
          make_planar_motion_noise());
        result.record_decision(
          FactorType::PlanarMotion, true, 1.0, "planar_motion_added");
      }});

  providers_.push_back(
    ProviderRegistration{
      FactorType::PlanarReference,
      [this](const FactorBuildContext & context, FactorBuildResult & result) {
        if (!parameters_.use_planar_prior) {
          result.record_decision(
            FactorType::PlanarReference, false, 1.0, "planar_prior_disabled");
          return;
        }
        add_factor<PlanarPriorFactor>(
          result,
          FactorType::PlanarReference,
          context.pose_key,
          parameters_.planar_reference_z,
          parameters_.planar_reference_roll,
          parameters_.planar_reference_pitch,
          make_planar_reference_noise());
        result.record_decision(
          FactorType::PlanarReference, true, 1.0, "planar_reference_added");
      }});
}

}  // namespace state_fusion_backend
