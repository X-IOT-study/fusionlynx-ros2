#pragma once

/**
 * @file fusion_backend.hpp
 * @brief ROS-independent GTSAM fixed-lag fusion core.
 * @author WenSheng Xu
 *
 * Modification history:
 * - 2026-08-21, WenSheng Xu:
 *   Added the first executable Pose3 fixed-lag backend for Wheel and FAST-LIO
 *   observations. MotionCore IMU and MAG remain health-monitoring inputs in
 *   v1; ROS callbacks stay outside this class for future factor adapters.
 * - 2026-08-21, WenSheng Xu:
 *   Kept velocity and bias types only in the result/API compatibility surface;
 *   v1 does not insert them into the graph until IMU preintegration is enabled.
 * - 2026-08-21, WenSheng Xu:
 *   Added wheel-derived stationary detection.  During a confirmed stop, a
 *   tight pose-hold factor prevents small FAST-LIO scan-matching oscillations
 *   from becoming visible as fused attitude motion.
 * - 2026-08-21, WenSheng Xu:
 *   Added a stationary anchor pose.  Holding every stationary frame against
 *   the immediately preceding estimate allowed a small random walk inside
 *   the fixed-lag smoother; the anchor keeps the confirmed stop tied to the
 *   pose at which the stop began and is released when wheel motion resumes.
 * - 2026-08-22, WenSheng Xu:
 *   Added the explicit FAST-LIO LiDAR-to-base extrinsic. FAST-LIO publishes
 *   the UniLidar LiDAR pose, while the fusion output contract is the
 *   FusionLynx base_link pose.
 * - 2026-08-22, WenSheng Xu:
 *   Changed FAST-LIO integration from repeated absolute pose priors to
 *   relative Pose3 between factors. The LIO frame alignment remains an
 *   initialization/recovery adapter and is not used to re-anchor every state.
 */

#include <optional>
#include <string>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include "state_fusion_backend/factor_types.hpp"
#include "state_fusion_backend/lio_health_monitor.hpp"

namespace state_fusion_backend
{

struct ImuSample
{
  double stamp{0.0};
  gtsam::Vector3 acceleration{gtsam::Vector3::Zero()};
  gtsam::Vector3 angular_velocity{gtsam::Vector3::Zero()};
};

struct WheelObservation
{
  double stamp{0.0};
  gtsam::Pose3 pose;
  gtsam::Vector3 linear_velocity{gtsam::Vector3::Zero()};
  gtsam::Vector3 angular_velocity{gtsam::Vector3::Zero()};
};

struct LioObservation
{
  double stamp{0.0};
  gtsam::Pose3 pose;
  // Stored in nav_msgs/Odometry order:
  // [translation_x, translation_y, translation_z,
  //  rotation_x, rotation_y, rotation_z].
  gtsam::Matrix6 covariance{gtsam::Matrix6::Zero()};
  bool covariance_valid{false};
};

struct MagObservation
{
  double stamp{0.0};
  gtsam::Vector3 field{gtsam::Vector3::Zero()};
};

struct FusionParameters
{
  double window_duration{8.0};
  double gravity{9.81};
  bool use_wheel_odom{true};
  bool use_fast_lio{true};
  bool use_lio_health{true};
  bool use_static_anchor{false};
  // Reserved for a future calibrated experiment. v1 keeps IMU and MAG
  // outside the localization graph and uses them only for LIO health.
  bool use_imu{false};
  bool use_mag{false};
  bool mag_calibration_valid{false};
  bool use_planar_prior{true};
  double planar_z_sigma{0.10};
  double planar_roll_sigma{0.10};
  double planar_pitch_sigma{0.10};
  double planar_reference_z{0.0};
  double planar_reference_roll{0.0};
  double planar_reference_pitch{0.0};
  double lio_z_sigma_scale{10.0};
  double wheel_x_sigma{0.03};
  double wheel_y_sigma{0.03};
  double wheel_yaw_sigma{0.05};
  double lio_noise_scale{1.0};
  double lio_position_sigma{0.05};
  double lio_rotation_sigma{0.03};
  double stationary_speed_threshold{0.02};
  double stationary_yaw_rate_threshold{0.03};
  double stationary_lio_scale{20.0};
  double stationary_pose_sigma{0.01};
  double zero_velocity_sigma{0.02};
  double fallback_pose_sigma{1.0};
  double imu_accel_sigma{0.10};
  double imu_gyro_sigma{0.03};
  double imu_bias_accel_sigma{0.001};
  double imu_bias_gyro_sigma{0.0001};
  double initial_pose_sigma{0.10};
  double initial_velocity_sigma{0.50};
  double initial_bias_sigma{0.10};
  double mag_noise_sigma{0.25};
  double mag_field_norm_min{20.0e-6};
  double mag_field_norm_max{70.0e-6};
  std::string robust_kernel{"huber"};
  double robust_kernel_scale{1.0};
  // T_lidar_base: FAST-LIO LiDAR coordinates -> FusionLynx base_link.
  gtsam::Pose3 lio_lidar_to_base;
};

enum class FusionBackendState
{
  WaitingForObservability,
  Initializing,
  Running,
  AligningRecovery
};

constexpr const char * fusion_backend_state_name(FusionBackendState state)
{
  switch (state) {
    case FusionBackendState::WaitingForObservability:
      return "waiting_for_observability";
    case FusionBackendState::Initializing:
      return "initializing";
    case FusionBackendState::Running:
      return "running";
    case FusionBackendState::AligningRecovery:
      return "aligning_recovery";
  }
  return "unknown";
}

struct FusionResult
{
  bool success{false};
  bool initialized{false};
  bool used_wheel{false};
  bool used_imu{false};
  bool used_mag{false};
  bool used_lio{false};
  bool stationary{false};
  double optimization_error{0.0};
  std::vector<FactorType> factors_used;
  FusionBackendState backend_state{FusionBackendState::WaitingForObservability};
  gtsam::Pose3 pose;
  gtsam::Vector3 velocity{gtsam::Vector3::Zero()};
  gtsam::Vector3 angular_velocity{gtsam::Vector3::Zero()};
  gtsam::imuBias::ConstantBias bias;
};

class FusionBackend final
{
public:
  explicit FusionBackend(const FusionParameters & parameters);
  ~FusionBackend();

  FusionResult update(
    const WheelObservation * wheel,
    const LioObservation * lio,
    const std::vector<ImuSample> & imu_samples,
    const MagObservation * mag,
    const LioHealthResult & health);

  FusionResult update(
    const WheelObservation * wheel,
    const LioObservation & lio,
    const std::vector<ImuSample> & imu_samples,
    const MagObservation * mag,
    const LioHealthResult & health)
  {
    return update(wheel, &lio, imu_samples, mag, health);
  }

  bool initialized() const {return initialized_;}
  FusionBackendState state() const {return state_;}
  void reset();

  std::optional<gtsam::Pose3> predicted_pose(
    const WheelObservation * wheel) const;
  std::optional<gtsam::Pose3> predicted_lio_pose(
    const WheelObservation * wheel) const;
  gtsam::Pose3 lio_pose_for_health(const gtsam::Pose3 & raw_pose) const;
  std::optional<gtsam::Pose3> wheel_delta(
    const WheelObservation * wheel) const;

private:
  gtsam::Pose3 convert_lio_pose_to_base(const gtsam::Pose3 & raw_pose) const;

  FusionParameters parameters_;
  std::unique_ptr<gtsam::IncrementalFixedLagSmoother> smoother_;
  std::unique_ptr<class FactorGraphBuilder> factor_builder_;

  bool initialized_{false};
  FusionBackendState state_{FusionBackendState::WaitingForObservability};
  std::size_t index_{0};
  double previous_stamp_{0.0};
  std::optional<WheelObservation> previous_wheel_;
  std::optional<LioObservation> previous_lio_;
  std::optional<gtsam::Pose3> previous_lio_pose_;
  gtsam::Pose3 lio_alignment_;
  std::optional<gtsam::Pose3> stationary_anchor_;
  bool lio_gap_{false};
};

}  // namespace state_fusion_backend
