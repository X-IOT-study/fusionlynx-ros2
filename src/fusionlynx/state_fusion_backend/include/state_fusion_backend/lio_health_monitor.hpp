#pragma once

/**
 * @file lio_health_monitor.hpp
 * @brief FAST-LIO output health and recovery state machine.
 * @author WenSheng Xu
 *
 * Modification history:
 * - 2026-08-21, WenSheng Xu:
 *   Added an external LIO supervisor. It evaluates only observable ROS
 *   outputs and does not pretend to be an internal FAST-LIO diagnostic API.
 * - 2026-08-22, WenSheng Xu:
 *   Added magnetic-field stability as a health-only cue. MAG does not create
 *   a localization factor in the v1 backend.
 * - 2026-08-23, WenSheng Xu:
 *   Changed timeout supervision to consume elapsed receive age instead of
 *   comparing unrelated sensor message timestamps.
 */

#include <string>
#include <optional>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>

namespace state_fusion_backend
{

enum class LioHealthState
{
  HEALTHY,
  DEGRADED,
  FAILED,
  RECOVERING
};

struct LioHealthResult
{
  LioHealthState state{LioHealthState::HEALTHY};
  double covariance_scale{1.0};
  bool accept_factor{true};
  bool requires_alignment{false};
  double translation_jump{0.0};
  double yaw_jump{0.0};
  double innovation_norm{0.0};
  double nis{0.0};
  double wheel_consistency{0.0};
  double imu_consistency{0.0};
  double mag_consistency{0.0};
  double confidence{1.0};
  bool static_detected{false};
  bool used_measurement_covariance{false};
  std::string reason{"initial"};
};

struct LioQualityContext
{
  bool prediction_valid{false};
  gtsam::Pose3 predicted_pose;
  bool wheel_valid{false};
  gtsam::Pose3 wheel_delta;
  bool imu_valid{false};
  double imu_yaw_delta{0.0};
  double imu_gyro_norm{0.0};
  bool static_candidate{false};
  bool mag_valid{false};
  double mag_field_norm{0.0};
  bool covariance_valid{false};
  gtsam::Matrix6 covariance{gtsam::Matrix6::Zero()};
  double fallback_position_sigma{0.10};
  double fallback_rotation_sigma{0.10};
  double lio_z_sigma_scale{10.0};
};

class LioHealthMonitor final
{
public:
  LioHealthMonitor(
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
    double mag_norm_relative_change_threshold);

  LioHealthResult observe(
    double stamp,
    const gtsam::Pose3 & pose,
    const LioQualityContext & context);
  LioHealthResult check_timeout(double elapsed_since_last_lio);
  void reset();

  LioHealthState state() const {return state_;}
  static const char * state_name(LioHealthState state);

private:
  double max_pose_jump_{1.0};
  double max_yaw_jump_{0.5};
  double failed_timeout_{0.5};
  double degraded_scale_{10.0};
  int recovery_stable_count_{5};
  double nis_degraded_threshold_{16.8};
  double nis_failed_threshold_{30.0};
  double consistency_degraded_threshold_{3.0};
  double consistency_failed_threshold_{6.0};
  double wheel_consistency_position_sigma_{0.10};
  double wheel_consistency_yaw_sigma_{0.08};
  double imu_consistency_yaw_sigma_{0.10};
  double static_min_duration_{0.5};
  double static_gyro_threshold_{0.03};
  double static_speed_threshold_{0.02};
  double static_yaw_rate_threshold_{0.03};
  double mag_norm_relative_change_threshold_{0.20};

  bool initialized_{false};
  double last_stamp_{0.0};
  gtsam::Pose3 last_pose_;
  int unstable_count_{0};
  int stable_count_{0};
  std::optional<double> static_since_;
  std::optional<double> last_mag_field_norm_;
  LioHealthState state_{LioHealthState::HEALTHY};
};

}  // namespace state_fusion_backend
