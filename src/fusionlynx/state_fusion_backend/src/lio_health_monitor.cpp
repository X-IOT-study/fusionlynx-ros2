#include "state_fusion_backend/lio_health_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace state_fusion_backend
{
namespace
{

double wrap_heading(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double safe_ratio(double value, double threshold)
{
  if (!std::isfinite(value) || !std::isfinite(threshold) || threshold <= 0.0) {
    return std::numeric_limits<double>::infinity();
  }
  return value / threshold;
}

bool finite_pose(const gtsam::Pose3 & pose)
{
  const auto translation = pose.translation();
  const auto rotation = pose.rotation().toQuaternion();
  return translation.array().isFinite().all() && rotation.coeffs().array().isFinite().all();
}

bool valid_covariance(const gtsam::Matrix6 & covariance)
{
  for (int i = 0; i < 6; ++i) {
    const double value = covariance(i, i);
    if (!std::isfinite(value) || value <= 1.0e-12 || value > 1.0e6) {
      return false;
    }
  }
  return true;
}

double covariance_diagonal(
  const LioQualityContext & context,
  int index)
{
  if (context.covariance_valid && valid_covariance(context.covariance)) {
    // LioQualityContext stores nav_msgs/Odometry covariance order:
    // [translation_x, translation_y, translation_z, rotation_x,
    //  rotation_y, rotation_z].  NIS uses Pose3::Logmap order:
    // [rotation_x, rotation_y, rotation_z, translation_x,
    //  translation_y, translation_z].
    constexpr int ros_index_for_gtsam[] = {3, 4, 5, 0, 1, 2};
    const int ros_index = ros_index_for_gtsam[index];
    double variance = std::max(context.covariance(ros_index, ros_index), 1.0e-8);
    // FAST-LIO's reported covariance is an internal filter estimate and can
    // be much smaller than the external system's repeatability. Apply the
    // configured floor to prevent centimeter-level residuals from becoming
    // artificial 100-sigma health failures.
    const double floor_sigma = index < 3 ?
      context.fallback_rotation_sigma : context.fallback_position_sigma;
    variance = std::max(variance, floor_sigma * floor_sigma);
    if (index == 5) {
      const double z_scale = std::max(context.lio_z_sigma_scale, 1.0);
      variance = std::max(variance, std::pow(floor_sigma * z_scale, 2.0));
      variance = std::max(
        variance,
        context.covariance(ros_index, ros_index) * z_scale * z_scale);
    }
    return variance;
  }
  const double fallback_sigma =
    index < 3 ? context.fallback_rotation_sigma : context.fallback_position_sigma;
  if (index == 5) {
    return std::pow(
      fallback_sigma * std::max(context.lio_z_sigma_scale, 1.0), 2.0);
  }
  return std::max(fallback_sigma * fallback_sigma, 1.0e-8);
}

}  // namespace

LioHealthMonitor::LioHealthMonitor(
  double max_pose_jump,
  double max_yaw_jump,
  double failed_timeout,
  double degraded_scale,
  int recovery_stable_count,
  double nis_degraded_threshold,
  double nis_failed_threshold,
  double consistency_degraded_threshold,
  double consistency_failed_threshold,
  double wheel_consistency_position_sigma,
  double wheel_consistency_yaw_sigma,
  double imu_consistency_yaw_sigma,
  double static_min_duration,
  double static_gyro_threshold,
  double static_speed_threshold,
  double static_yaw_rate_threshold,
  double mag_norm_relative_change_threshold)
: max_pose_jump_(max_pose_jump),
  max_yaw_jump_(max_yaw_jump),
  failed_timeout_(failed_timeout),
  degraded_scale_(degraded_scale),
  recovery_stable_count_(std::max(1, recovery_stable_count)),
  nis_degraded_threshold_(std::max(nis_degraded_threshold, 1.0)),
  nis_failed_threshold_(std::max(nis_failed_threshold, nis_degraded_threshold_ + 1.0)),
  consistency_degraded_threshold_(std::max(consistency_degraded_threshold, 1.0)),
  consistency_failed_threshold_(
    std::max(consistency_failed_threshold, consistency_degraded_threshold_ + 1.0)),
  wheel_consistency_position_sigma_(std::max(wheel_consistency_position_sigma, 1.0e-4)),
  wheel_consistency_yaw_sigma_(std::max(wheel_consistency_yaw_sigma, 1.0e-4)),
  imu_consistency_yaw_sigma_(std::max(imu_consistency_yaw_sigma, 1.0e-4)),
  static_min_duration_(std::max(static_min_duration, 0.0)),
  static_gyro_threshold_(std::max(static_gyro_threshold, 0.0)),
  static_speed_threshold_(std::max(static_speed_threshold, 0.0)),
  static_yaw_rate_threshold_(std::max(static_yaw_rate_threshold, 0.0)),
  mag_norm_relative_change_threshold_(
    std::max(mag_norm_relative_change_threshold, 1.0e-4))
{
}

LioHealthResult LioHealthMonitor::observe(
  double stamp,
  const gtsam::Pose3 & pose,
  const LioQualityContext & context)
{
  LioHealthResult result;
  result.state = state_;
  result.covariance_scale = 1.0;
  result.used_measurement_covariance = context.covariance_valid;

  const bool static_candidate =
    context.static_candidate &&
    context.wheel_valid &&
    context.imu_valid &&
    context.imu_gyro_norm <= static_gyro_threshold_;
  if (static_candidate) {
    if (!static_since_) {
      static_since_ = stamp;
    }
    result.static_detected =
      std::isfinite(stamp) && stamp - *static_since_ >= static_min_duration_;
  } else {
    static_since_.reset();
  }

  if (!std::isfinite(stamp) || stamp <= 0.0 || !finite_pose(pose)) {
    result.state = LioHealthState::FAILED;
    result.covariance_scale = degraded_scale_;
    result.accept_factor = false;
    result.confidence = 0.0;
    result.reason = "invalid LIO timestamp or pose";
    state_ = result.state;
    return result;
  }

  if (!initialized_) {
    initialized_ = true;
    last_stamp_ = stamp;
    last_pose_ = pose;
    result.reason = "initialized";
    return result;
  }

  const double dt = stamp - last_stamp_;
  const gtsam::Pose3 delta = last_pose_.between(pose);
  const double translation_jump = delta.translation().norm();
  const double yaw_jump = std::abs(delta.rotation().yaw());
  result.translation_jump = translation_jump;
  result.yaw_jump = yaw_jump;

  const bool use_absolute_innovation =
    context.prediction_valid && state_ != LioHealthState::FAILED;
  if (use_absolute_innovation) {
    const gtsam::Vector6 innovation =
      gtsam::Pose3::Logmap(context.predicted_pose.between(pose));
    result.innovation_norm = innovation.norm();
    for (int i = 0; i < 6; ++i) {
      const double variance = covariance_diagonal(context, i);
      result.nis += innovation(i) * innovation(i) / variance;
    }
  }

  if (context.wheel_valid) {
    const gtsam::Vector6 wheel_error =
      gtsam::Pose3::Logmap(context.wheel_delta.inverse().compose(delta));
    // Pose3::Logmap order is [roll, pitch, yaw, x, y, z].
    result.wheel_consistency = std::sqrt(
      std::pow(wheel_error(3) / wheel_consistency_position_sigma_, 2.0) +
      std::pow(wheel_error(4) / wheel_consistency_position_sigma_, 2.0) +
      std::pow(wheel_error(2) / wheel_consistency_yaw_sigma_, 2.0));
  }

  if (context.imu_valid) {
    const double lio_yaw_delta = delta.rotation().yaw();
    result.imu_consistency = std::abs(
      wrap_heading(lio_yaw_delta - context.imu_yaw_delta)) /
      imu_consistency_yaw_sigma_;
  }

  bool mag_consistency_valid = false;
  if (context.mag_valid && std::isfinite(context.mag_field_norm)) {
    if (last_mag_field_norm_) {
      const double relative_change =
        std::abs(context.mag_field_norm - *last_mag_field_norm_) /
        std::max(*last_mag_field_norm_, 1.0e-9);
      result.mag_consistency =
        relative_change / mag_norm_relative_change_threshold_;
      mag_consistency_valid = true;
    }
    last_mag_field_norm_ = context.mag_field_norm;
  } else {
    last_mag_field_norm_.reset();
  }

  const double nis_ratio =
    use_absolute_innovation ? safe_ratio(result.nis, nis_degraded_threshold_) : 0.0;
  const double wheel_ratio =
    context.wheel_valid ?
    safe_ratio(result.wheel_consistency, consistency_degraded_threshold_) : 0.0;
  const double imu_ratio =
    context.imu_valid ?
    safe_ratio(result.imu_consistency, consistency_degraded_threshold_) : 0.0;
  const double mag_ratio =
    mag_consistency_valid ? result.mag_consistency : 0.0;
  const double quality_ratio =
    std::max({nis_ratio, wheel_ratio, imu_ratio, mag_ratio});
  result.confidence = 1.0 / (1.0 + std::max(quality_ratio - 1.0, 0.0));

  const bool invalid_interval = !std::isfinite(dt) || dt <= 0.0 || dt > failed_timeout_;
  const bool jump_detected =
    translation_jump > max_pose_jump_ || yaw_jump > max_yaw_jump_;
  const bool failed_consistency =
    (use_absolute_innovation && result.nis > nis_failed_threshold_) ||
    (context.wheel_valid && result.wheel_consistency > consistency_failed_threshold_) ||
    (context.imu_valid && result.imu_consistency > consistency_failed_threshold_);
  const bool degraded_consistency =
    (use_absolute_innovation && result.nis > nis_degraded_threshold_) ||
    (context.wheel_valid &&
    result.wheel_consistency > consistency_degraded_threshold_) ||
    (context.imu_valid &&
    result.imu_consistency > consistency_degraded_threshold_);

  if (invalid_interval || jump_detected || failed_consistency) {
    ++unstable_count_;
    stable_count_ = 0;
    if (unstable_count_ >= 2 || invalid_interval) {
      state_ = LioHealthState::FAILED;
      result.accept_factor = false;
      result.reason = invalid_interval ? "stale or invalid interval" :
        (jump_detected ? "gross pose jump" : "innovation or sensor inconsistency");
    } else {
      state_ = LioHealthState::DEGRADED;
      result.reason = jump_detected ? "gross pose jump candidate" :
        "innovation or sensor inconsistency candidate";
    }
  } else if (degraded_consistency) {
    unstable_count_ = 0;
    stable_count_ = 0;
    state_ = LioHealthState::DEGRADED;
    result.reason = "LIO innovation degraded";
  } else {
    unstable_count_ = 0;
    ++stable_count_;
    if (state_ == LioHealthState::FAILED) {
      state_ = LioHealthState::RECOVERING;
      result.requires_alignment = true;
      result.reason = "stable output after failure";
    } else if (
      state_ == LioHealthState::RECOVERING &&
      stable_count_ >= recovery_stable_count_)
    {
      state_ = LioHealthState::HEALTHY;
      result.reason = "recovery complete";
    } else if (state_ == LioHealthState::DEGRADED) {
      state_ = LioHealthState::RECOVERING;
      result.reason = "degraded output recovered";
    } else {
      state_ = LioHealthState::HEALTHY;
      result.reason = "healthy";
    }
  }

  last_stamp_ = stamp;
  last_pose_ = pose;
  result.state = state_;
  const double quality_scale = std::clamp(
    std::max(1.0, quality_ratio),
    1.0,
    std::max(degraded_scale_, 1.0));
  result.covariance_scale = quality_scale;
  if (state_ == LioHealthState::DEGRADED ||
    state_ == LioHealthState::RECOVERING)
  {
    result.covariance_scale = std::max(result.covariance_scale, degraded_scale_);
  }
  result.accept_factor = state_ != LioHealthState::FAILED;
  if (state_ == LioHealthState::RECOVERING) {
    result.covariance_scale = std::max(result.covariance_scale, degraded_scale_);
  }
  return result;
}

LioHealthResult LioHealthMonitor::check_timeout(double elapsed_since_last_lio)
{
  LioHealthResult result;
  result.state = state_;
  result.covariance_scale = state_ == LioHealthState::HEALTHY ? 1.0 : degraded_scale_;
  result.accept_factor = state_ != LioHealthState::FAILED;
  if (
    initialized_ &&
    std::isfinite(elapsed_since_last_lio) &&
    elapsed_since_last_lio > failed_timeout_)
  {
    state_ = LioHealthState::FAILED;
    stable_count_ = 0;
    ++unstable_count_;
    result.state = LioHealthState::FAILED;
    result.covariance_scale = degraded_scale_;
    result.accept_factor = false;
    result.reason = "FAST-LIO timeout";
  } else {
    result.reason = "FAST-LIO timeout not reached";
  }
  return result;
}

void LioHealthMonitor::reset()
{
  initialized_ = false;
  last_stamp_ = 0.0;
  last_pose_ = gtsam::Pose3();
  unstable_count_ = 0;
  stable_count_ = 0;
  static_since_.reset();
  last_mag_field_norm_.reset();
  state_ = LioHealthState::HEALTHY;
}

const char * LioHealthMonitor::state_name(LioHealthState state)
{
  switch (state) {
    case LioHealthState::HEALTHY:
      return "HEALTHY";
    case LioHealthState::DEGRADED:
      return "DEGRADED";
    case LioHealthState::FAILED:
      return "FAILED";
    case LioHealthState::RECOVERING:
      return "RECOVERING";
  }
  return "UNKNOWN";
}

}  // namespace state_fusion_backend
