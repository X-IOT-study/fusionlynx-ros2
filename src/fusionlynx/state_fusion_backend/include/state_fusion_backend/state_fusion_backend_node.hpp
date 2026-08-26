#pragma once

/**
 * @file state_fusion_backend_node.hpp
 * @brief Lifecycle ROS 2 wrapper for the FusionLynx factor-graph backend.
 * @author WenSheng Xu
 *
 * Modification history:
 * - 2026-08-21, WenSheng Xu:
 *   Added subscriptions using the topic names already published by the
 *   current MotionCore and FAST-LIO packages.
 * - 2026-08-21, WenSheng Xu:
 *   Changed the final fused odometry topic to the standard `/odom` name.
 *   MotionCore `/motioncore/...` names remain reserved for driver and
 *   preprocessing outputs.
 * - 2026-08-21, WenSheng Xu:
 *   Added monotonic FAST-LIO timestamp gating and failed-update graph reset
 *   handling to prevent duplicate keys after a GTSAM exception.
 * - 2026-08-21, WenSheng Xu:
 *   Added wheel twist thresholds used to identify a confirmed stationary
 *   robot before applying the stationary pose-hold constraint.
 * - 2026-08-22, WenSheng Xu:
 *   Added configurable FAST-LIO LiDAR-to-base extrinsic parameters so the
 *   final odom pose is expressed in the physical FusionLynx base_link frame.
 * - 2026-08-22, WenSheng Xu:
 *   Added wheel-only prediction publishing between FAST-LIO updates. This
 *   keeps `/odom` responsive during LIO gaps without adding a second graph
 *   or TF authority.
 * - 2026-08-22, WenSheng Xu:
 *   Added a bounded Wheel Odom history so an LIO callback does not discard
 *   the wheel factor merely because the newest wheel sample is slightly
 *   newer than the LIO timestamp.
 * - 2026-08-23, WenSheng Xu:
 *   Changed the LIO watchdog to use steady-clock receive age rather than
 *   comparing FAST-LIO and Wheel message timestamps.
 */

#include <array>
#include <chrono>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "state_fusion_backend/fusion_backend.hpp"
#include "state_fusion_backend/lio_health_monitor.hpp"

namespace state_fusion_backend
{

class StateFusionBackendNode final : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit StateFusionBackendNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~StateFusionBackendNode() override;

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

private:
  struct Parameters
  {
    std::string wheel_topic{"/motioncore/wheel_odom"};
    std::string imu_topic{"/motioncore/imu/data"};
    std::string mag_topic{"/motioncore/imu/mag"};
    std::string lio_topic{"/fast_lio/odom"};
    // Modification: 2026-08-21, WenSheng Xu.
    // Reason: `/odom` is the authoritative fused-odometry interface; the
    // `/motioncore/*` namespace is reserved for MotionCore-layer outputs.
    std::string odom_topic{"/odom"};
    std::string diagnostics_topic{"/motioncore/diagnostics"};
    std::string odom_frame{"odom"};
    std::string base_link_frame{"base_link"};
    std::string wheel_odom_frame{"odom_wheel"};
    std::string wheel_child_frame{"base_link"};
    std::string lio_world_frame{"camera_init"};
    std::string lio_child_frame{"lidar_link"};
    std::string fusion_profile{"D"};
    bool use_wheel_odom{true};
    bool use_fast_lio{true};
    bool use_lio_health{false};
    bool use_static_anchor{false};
    std::string robust_kernel{"huber"};
    double frequency{50.0};
    double sensor_timeout{0.5};
    double max_time_delay{0.5};
    double max_future_stamp{0.25};
    double max_pose_jump{1.0};
    double max_pose_jump_yaw{0.5};
    double lio_degraded_scale{10.0};
    int recovery_stable_count{5};
    double lio_nis_degraded_threshold{16.8};
    double lio_nis_failed_threshold{30.0};
    double lio_consistency_degraded_threshold{3.0};
    double lio_consistency_failed_threshold{6.0};
    double wheel_consistency_position_sigma{0.10};
    double wheel_consistency_yaw_sigma{0.08};
    double imu_consistency_yaw_sigma{0.10};
    double window_duration{8.0};
    double gravity{9.81};
    // Kept for configuration compatibility. v1 uses IMU/MAG for LIO health
    // only; these do not enable direct localization factors.
    bool use_motioncore_imu{false};
    bool use_magnetic_heading_factor{false};
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
    double lio_position_sigma{0.05};
    double lio_rotation_sigma{0.03};
    double stationary_speed_threshold{0.02};
    double stationary_yaw_rate_threshold{0.03};
    double static_min_duration{0.5};
    double static_gyro_threshold{0.03};
    double mag_norm_relative_change_threshold{0.20};
    double stationary_lio_scale{20.0};
    double stationary_pose_sigma{0.01};
    double zero_velocity_sigma{0.02};
    double fallback_pose_sigma{1.0};
    double imu_accel_sigma{0.10};
    double imu_gyro_sigma{0.03};
    double imu_bias_accel_sigma{0.001};
    double imu_bias_gyro_sigma{0.0001};
    double mag_noise_sigma{0.25};
    double mag_field_norm_min{20.0e-6};
    double mag_field_norm_max{70.0e-6};
    double robust_kernel_scale{1.0};
    int queue_size{2000};
    double reorder_window{0.10};
    double clock_offset_tolerance{0.25};
    double diagnostics_publish_rate{5.0};
    bool publish_tf{true};
    std::vector<double> lio_lidar_to_base_xyz{
      -0.00312585, 0.00736404, -0.12};
    std::vector<double> lio_lidar_to_base_rpy{
      0.0, 0.0, -1.1693706};
  };

  void declare_parameters();
  bool load_parameters();
  void create_entities();
  void destroy_entities();
  void handle_wheel(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void handle_imu(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void handle_mag(const sensor_msgs::msg::MagneticField::ConstSharedPtr msg);
  void handle_lio(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void process_pending();
  void publish_wheel_prediction();
  void publish_result(
    const rclcpp::Time & stamp,
    const FusionResult & result,
    const LioHealthResult & health);
  void publish_diagnostics(
    const rclcpp::Time & stamp,
    const FusionResult & result,
    const LioHealthResult & health,
    const std::string & summary,
    std::uint8_t level);
  void log_lio_state_transition(
    const rclcpp::Time & stamp,
    const LioHealthResult & health);

  static gtsam::Pose3 to_pose(const geometry_msgs::msg::Pose & pose);
  static gtsam::Matrix6 to_covariance(
    const std::array<double, 36> & covariance,
    bool * valid);
  static geometry_msgs::msg::Pose to_pose_message(const gtsam::Pose3 & pose);
  static gtsam::Vector3 to_vector(const geometry_msgs::msg::Vector3 & value);

  Parameters parameters_;
  std::unique_ptr<FusionBackend> backend_;
  std::unique_ptr<LioHealthMonitor> health_monitor_;

  std::optional<WheelObservation> latest_wheel_;
  std::deque<WheelObservation> wheel_buffer_;
  std::deque<LioObservation> lio_buffer_;
  std::optional<MagObservation> latest_mag_;
  std::deque<ImuSample> imu_buffer_;
  std::optional<double> last_processed_imu_stamp_;
  std::optional<double> last_processed_wheel_stamp_;
  std::optional<double> last_processed_lio_stamp_;
  std::optional<double> last_processed_event_stamp_;
  std::optional<double> last_received_lio_stamp_;
  std::optional<rclcpp::Time> last_lio_message_stamp_;
  std::optional<std::chrono::steady_clock::time_point> last_lio_receive_time_;
  mutable std::mutex mutex_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lio_sub_;
  rclcpp::TimerBase::SharedPtr wheel_prediction_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  std::optional<FusionResult> last_result_;
  std::optional<LioHealthResult> last_health_;
  std::optional<LioHealthState> last_logged_lio_state_;
  std::int64_t last_diagnostics_stamp_ns_{0};
  std::int64_t last_output_stamp_ns_{0};
};

}  // namespace state_fusion_backend
