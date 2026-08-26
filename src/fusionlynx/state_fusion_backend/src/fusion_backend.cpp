#include "state_fusion_backend/fusion_backend.hpp"

#include <cmath>

#include <gtsam/inference/Symbol.h>

#include "state_fusion_backend/factor_graph_builder.hpp"

namespace state_fusion_backend
{
namespace
{

FactorReliabilityState to_factor_reliability_state(LioHealthState state)
{
  switch (state) {
    case LioHealthState::HEALTHY:
      return FactorReliabilityState::Reliable;
    case LioHealthState::DEGRADED:
      return FactorReliabilityState::Degraded;
    case LioHealthState::FAILED:
      return FactorReliabilityState::Failed;
    case LioHealthState::RECOVERING:
      return FactorReliabilityState::Recovering;
  }
  return FactorReliabilityState::Failed;
}

}  // namespace

FusionBackend::FusionBackend(const FusionParameters & parameters)
: parameters_(parameters),
  smoother_(std::make_unique<gtsam::IncrementalFixedLagSmoother>(parameters.window_duration)),
  factor_builder_(std::make_unique<FactorGraphBuilder>(parameters))
{
}

FusionBackend::~FusionBackend() = default;

gtsam::Pose3 FusionBackend::convert_lio_pose_to_base(
  const gtsam::Pose3 & raw_pose) const
{
  // FAST-LIO publishes T_camera_init_lidar. The fixed extrinsic is
  // T_lidar_base, therefore the graph receives T_camera_init_base.
  return raw_pose.compose(parameters_.lio_lidar_to_base);
}

std::optional<gtsam::Pose3> FusionBackend::predicted_pose(
  const WheelObservation * wheel) const
{
  if (!initialized_ || index_ == 0) {
    return std::nullopt;
  }
  const gtsam::Pose3 previous_pose =
    smoother_->calculateEstimate<gtsam::Pose3>(gtsam::Symbol('x', index_ - 1));
  if (wheel != nullptr && previous_wheel_) {
    return previous_pose.compose(previous_wheel_->pose.between(wheel->pose));
  }
  return previous_pose;
}

std::optional<gtsam::Pose3> FusionBackend::predicted_lio_pose(
  const WheelObservation * wheel) const
{
  const auto prediction = predicted_pose(wheel);
  if (!prediction) {
    return std::nullopt;
  }
  return lio_alignment_.inverse().compose(*prediction);
}

gtsam::Pose3 FusionBackend::lio_pose_for_health(const gtsam::Pose3 & raw_pose) const
{
  return convert_lio_pose_to_base(raw_pose);
}

std::optional<gtsam::Pose3> FusionBackend::wheel_delta(
  const WheelObservation * wheel) const
{
  if (wheel == nullptr || !previous_wheel_) {
    return std::nullopt;
  }
  return previous_wheel_->pose.between(wheel->pose);
}

FusionResult FusionBackend::update(
  const WheelObservation * wheel,
  const LioObservation * lio,
  const std::vector<ImuSample> & imu_samples,
  const MagObservation * mag,
  const LioHealthResult & health)
{
  // IMU and MAG are consumed by LioHealthMonitor in v1. Keep these
  // arguments in the backend API for a future calibrated factor adapter.
  (void)imu_samples;
  (void)mag;

  FusionResult result;
  result.backend_state = state_;
  const WheelObservation * accepted_wheel =
    parameters_.use_wheel_odom ? wheel : nullptr;
  const LioObservation * accepted_lio =
    parameters_.use_fast_lio ? lio : nullptr;

  if (accepted_wheel == nullptr && accepted_lio == nullptr) {
    state_ = FusionBackendState::WaitingForObservability;
    result.backend_state = state_;
    result.pose = gtsam::Pose3();
    result.initialized = initialized_;
    return result;
  }

  const double event_stamp = accepted_lio != nullptr ?
    accepted_lio->stamp : accepted_wheel->stamp;
  if (!std::isfinite(event_stamp) || event_stamp <= 0.0) {
    result.pose = initialized_ ? smoother_->calculateEstimate<gtsam::Pose3>(
      gtsam::Symbol('x', index_ - 1)) : gtsam::Pose3();
    result.initialized = initialized_;
    return result;
  }

  const gtsam::Pose3 lio_base_pose = accepted_lio != nullptr ?
    convert_lio_pose_to_base(accepted_lio->pose) : gtsam::Pose3();
  const bool lio_factor_allowed =
    accepted_lio != nullptr && health.accept_factor;
  if (!initialized_) {
    state_ = FusionBackendState::Initializing;
    if (accepted_lio != nullptr && lio_factor_allowed) {
      lio_alignment_ = lio_base_pose.inverse();
    }
  } else if (accepted_lio != nullptr && (health.requires_alignment || lio_gap_)) {
    const gtsam::Pose3 previous_pose =
      smoother_->calculateEstimate<gtsam::Pose3>(gtsam::Symbol('x', index_ - 1));
    lio_alignment_ = previous_pose.compose(lio_base_pose.inverse());
  }
  const gtsam::Pose3 aligned_lio =
    accepted_lio != nullptr ? lio_alignment_.compose(lio_base_pose) :
    gtsam::Pose3();
  const gtsam::Symbol pose_key('x', index_);

  if (!initialized_) {
    // Match the validation bridge: convert FAST-LIO's lidar pose to
    // base_link first, then express the first accepted pose at the local
    // odometry origin through lio_alignment_.  The first observation is an
    // initializer, not a relative factor.
    const gtsam::Pose3 initial_pose =
      accepted_lio != nullptr && lio_factor_allowed ?
      aligned_lio : gtsam::Pose3();
    const auto build_result = factor_builder_->build_initialization(
      pose_key, initial_pose, event_stamp);
    smoother_->update(
      build_result.factors,
      build_result.values,
      build_result.timestamps);
    initialized_ = true;
    previous_stamp_ = event_stamp;
    if (accepted_wheel != nullptr) {
      previous_wheel_ = *accepted_wheel;
    }
    if (accepted_lio != nullptr && lio_factor_allowed) {
      previous_lio_ = *accepted_lio;
      previous_lio_pose_ = lio_base_pose;
      lio_gap_ = false;
    } else if (parameters_.use_fast_lio) {
      lio_gap_ = true;
    }
    ++index_;
    const auto estimate = smoother_->calculateEstimate();
    result.pose = estimate.at<gtsam::Pose3>(pose_key);
    result.velocity = accepted_wheel != nullptr ?
      result.pose.rotation().rotate(accepted_wheel->linear_velocity) :
      gtsam::Vector3::Zero();
    result.angular_velocity = accepted_wheel != nullptr ?
      accepted_wheel->angular_velocity : gtsam::Vector3::Zero();
    result.bias = gtsam::imuBias::ConstantBias();
    result.optimization_error = smoother_->getFactors().error(estimate);
    result.success = true;
    result.initialized = true;
    state_ = FusionBackendState::Running;
    result.backend_state = state_;
    result.factors_used = build_result.factor_types;
    // No motion factor is inserted for the initializer itself.  These flags
    // describe factors used by this update, not observations cached for the
    // next relative increment.
    result.used_wheel = false;
    result.used_lio = false;
    return result;
  }

  const gtsam::Symbol previous_pose_key('x', index_ - 1);
  const auto previous_pose = smoother_->calculateEstimate<gtsam::Pose3>(previous_pose_key);

  const bool stationary = parameters_.use_static_anchor && health.static_detected;

  // Modification: 2026-08-21, WenSheng Xu.
  // Reason: A previous-pose stationary factor is relative to a moving
  // estimate.  FAST-LIO can then move that estimate a small amount on every
  // scan and the fixed-lag smoother can expose the movement as attitude
  // jitter.  Capture one soft anchor when the wheel odometry first confirms
  // a stop.  This is still a finite-covariance factor, not a hard lock.
  if (stationary) {
    if (!stationary_anchor_) {
      stationary_anchor_ = previous_pose;
    }
  } else {
    stationary_anchor_.reset();
  }

  // The first stable frame after a LIO failure is used only to establish the
  // new frame alignment. It must not also create a relative factor from the
  // pre-failure raw pose, otherwise the recovery jump is reintroduced into
  // the graph.
  const bool recovery_alignment_frame =
    accepted_lio != nullptr && (health.requires_alignment || lio_gap_);
  if (!health.accept_factor || recovery_alignment_frame) {
    state_ = FusionBackendState::AligningRecovery;
  }
  const bool use_lio =
    accepted_lio != nullptr && health.accept_factor && !recovery_alignment_frame;
  const bool has_wheel_prediction =
    accepted_wheel != nullptr && previous_wheel_;
  const gtsam::Pose3 initial_pose =
    (accepted_lio != nullptr && health.accept_factor && !recovery_alignment_frame) ?
    aligned_lio :
    (has_wheel_prediction ?
    previous_pose.compose(previous_wheel_->pose.between(accepted_wheel->pose)) :
    previous_pose);
  FactorBuildContext build_context;
  build_context.previous_pose_key = previous_pose_key;
  build_context.pose_key = pose_key;
  build_context.stamp = event_stamp;
  build_context.previous_pose = previous_pose;
  build_context.initial_pose = initial_pose;
  build_context.use_lio = use_lio;
  build_context.aligned_lio = aligned_lio;
  build_context.lio_scale = health.covariance_scale *
    (stationary ? parameters_.stationary_lio_scale : 1.0);
  build_context.lio_reliability = FactorReliabilityDecision{
    to_factor_reliability_state(health.state),
    health.covariance_scale,
    health.accept_factor,
    health.reason};
  build_context.stationary = stationary;
  build_context.stationary_anchor = stationary_anchor_;

  if (accepted_wheel != nullptr && previous_wheel_) {
    build_context.wheel_delta =
      previous_wheel_->pose.between(accepted_wheel->pose);
  }
  if (accepted_lio != nullptr && use_lio) {
    build_context.lio = *accepted_lio;
    build_context.previous_lio = previous_lio_;
    if (previous_lio_pose_) {
      // FAST-LIO is a local odometry frontend. Use its incremental motion
      // as a relative factor; alignment is only for the initial value and
      // recovery continuity.
      build_context.lio_delta = previous_lio_pose_->between(lio_base_pose);
    }
  }

  const auto build_result = factor_builder_->build_motion(build_context);
  smoother_->update(
    build_result.factors,
    build_result.values,
    build_result.timestamps);
  const auto estimate = smoother_->calculateEstimate();
  result.pose = estimate.at<gtsam::Pose3>(pose_key);
  result.velocity = accepted_wheel != nullptr ?
    result.pose.rotation().rotate(accepted_wheel->linear_velocity) :
    gtsam::Vector3::Zero();
  result.angular_velocity = accepted_wheel != nullptr ?
    accepted_wheel->angular_velocity : gtsam::Vector3::Zero();
  result.bias = gtsam::imuBias::ConstantBias();
  result.optimization_error = smoother_->getFactors().error(estimate);
  result.success = true;
  result.initialized = true;
  result.factors_used = build_result.factor_types;
  result.used_wheel = build_result.used_wheel;
  result.used_imu = false;
  result.used_mag = false;
  result.used_lio = build_result.used_lio;
  result.stationary = stationary;

  previous_stamp_ = event_stamp;
  if (accepted_wheel != nullptr) {
    previous_wheel_ = *accepted_wheel;
  }
  if (accepted_lio != nullptr) {
    if (use_lio || recovery_alignment_frame) {
      previous_lio_ = *accepted_lio;
      previous_lio_pose_ = lio_base_pose;
    }
    // A recovery-alignment frame closes the gap. If the health monitor still
    // rejects this measurement, keep the gap open until a stable frame arrives.
    lio_gap_ = !health.accept_factor;
    state_ = health.accept_factor && !lio_gap_ ?
      FusionBackendState::Running : FusionBackendState::AligningRecovery;
  } else if (parameters_.use_fast_lio) {
    lio_gap_ = true;
    state_ = FusionBackendState::AligningRecovery;
  } else {
    state_ = FusionBackendState::Running;
  }
  ++index_;
  result.backend_state = state_;
  return result;
}

void FusionBackend::reset()
{
  smoother_ = std::make_unique<gtsam::IncrementalFixedLagSmoother>(parameters_.window_duration);
  initialized_ = false;
  state_ = FusionBackendState::WaitingForObservability;
  index_ = 0;
  previous_stamp_ = 0.0;
  previous_wheel_.reset();
  previous_lio_.reset();
  previous_lio_pose_.reset();
  lio_alignment_ = gtsam::Pose3();
  stationary_anchor_.reset();
  lio_gap_ = false;
}

}  // namespace state_fusion_backend
