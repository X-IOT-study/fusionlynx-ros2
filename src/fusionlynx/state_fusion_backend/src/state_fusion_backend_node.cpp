#include "state_fusion_backend/state_fusion_backend_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

#include <diagnostic_msgs/msg/key_value.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gtsam/geometry/Rot3.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace state_fusion_backend
{
namespace
{

using namespace std::chrono_literals;

}  // namespace

StateFusionBackendNode::StateFusionBackendNode(const rclcpp::NodeOptions & options)
: LifecycleNode("state_fusion_backend_node", options)
{
  declare_parameters();
}

StateFusionBackendNode::~StateFusionBackendNode()
{
  destroy_entities();
}

void StateFusionBackendNode::declare_parameters()
{
  declare_parameter("wheel_odom_topic", parameters_.wheel_topic);
  declare_parameter("imu_topic", parameters_.imu_topic);
  declare_parameter("mag_topic", parameters_.mag_topic);
  declare_parameter("fastlio_topic", parameters_.lio_topic);
  declare_parameter("fused_odom_topic", parameters_.odom_topic);
  declare_parameter("diagnostics_topic", parameters_.diagnostics_topic);
  declare_parameter("odom_frame", parameters_.odom_frame);
  declare_parameter("base_link_frame", parameters_.base_link_frame);
  declare_parameter("wheel_odom_frame", parameters_.wheel_odom_frame);
  declare_parameter("wheel_child_frame", parameters_.wheel_child_frame);
  declare_parameter("lio_world_frame", parameters_.lio_world_frame);
  declare_parameter("lio_child_frame", parameters_.lio_child_frame);
  declare_parameter("fusion_profile", parameters_.fusion_profile);
  declare_parameter("use_wheel_odom", parameters_.use_wheel_odom);
  declare_parameter("use_fast_lio", parameters_.use_fast_lio);
  declare_parameter("use_lio_health", parameters_.use_lio_health);
  declare_parameter("use_static_anchor", parameters_.use_static_anchor);
  declare_parameter("robust_kernel", parameters_.robust_kernel);
  declare_parameter("frequency", parameters_.frequency);
  declare_parameter("sensor_timeout", parameters_.sensor_timeout);
  declare_parameter("max_time_delay", parameters_.max_time_delay);
  declare_parameter("max_future_stamp", parameters_.max_future_stamp);
  declare_parameter("max_pose_jump", parameters_.max_pose_jump);
  declare_parameter("max_pose_jump_yaw", parameters_.max_pose_jump_yaw);
  declare_parameter("lio_degraded_scale", parameters_.lio_degraded_scale);
  declare_parameter("recovery_stable_count", parameters_.recovery_stable_count);
  declare_parameter(
    "lio_nis_degraded_threshold", parameters_.lio_nis_degraded_threshold);
  declare_parameter("lio_nis_failed_threshold", parameters_.lio_nis_failed_threshold);
  declare_parameter(
    "lio_consistency_degraded_threshold",
    parameters_.lio_consistency_degraded_threshold);
  declare_parameter(
    "lio_consistency_failed_threshold",
    parameters_.lio_consistency_failed_threshold);
  declare_parameter(
    "wheel_consistency_position_sigma",
    parameters_.wheel_consistency_position_sigma);
  declare_parameter(
    "wheel_consistency_yaw_sigma",
    parameters_.wheel_consistency_yaw_sigma);
  declare_parameter(
    "imu_consistency_yaw_sigma", parameters_.imu_consistency_yaw_sigma);
  declare_parameter("window_duration", parameters_.window_duration);
  declare_parameter("gravity", parameters_.gravity);
  declare_parameter("use_motioncore_imu", parameters_.use_motioncore_imu);
  declare_parameter("use_magnetic_heading_factor", parameters_.use_magnetic_heading_factor);
  declare_parameter("mag_calibration_valid", parameters_.mag_calibration_valid);
  declare_parameter("use_planar_prior", parameters_.use_planar_prior);
  declare_parameter("planar_z_sigma", parameters_.planar_z_sigma);
  declare_parameter("planar_roll_sigma", parameters_.planar_roll_sigma);
  declare_parameter("planar_pitch_sigma", parameters_.planar_pitch_sigma);
  declare_parameter("planar_reference_z", parameters_.planar_reference_z);
  declare_parameter("planar_reference_roll", parameters_.planar_reference_roll);
  declare_parameter("planar_reference_pitch", parameters_.planar_reference_pitch);
  declare_parameter("lio_z_sigma_scale", parameters_.lio_z_sigma_scale);
  declare_parameter("wheel_x_sigma", parameters_.wheel_x_sigma);
  declare_parameter("wheel_y_sigma", parameters_.wheel_y_sigma);
  declare_parameter("wheel_yaw_sigma", parameters_.wheel_yaw_sigma);
  declare_parameter("lio_position_sigma", parameters_.lio_position_sigma);
  declare_parameter("lio_rotation_sigma", parameters_.lio_rotation_sigma);
  declare_parameter("stationary_speed_threshold", parameters_.stationary_speed_threshold);
  declare_parameter("stationary_yaw_rate_threshold", parameters_.stationary_yaw_rate_threshold);
  declare_parameter("static_min_duration", parameters_.static_min_duration);
  declare_parameter("static_gyro_threshold", parameters_.static_gyro_threshold);
  declare_parameter(
    "mag_norm_relative_change_threshold",
    parameters_.mag_norm_relative_change_threshold);
  declare_parameter("stationary_lio_scale", parameters_.stationary_lio_scale);
  declare_parameter("stationary_pose_sigma", parameters_.stationary_pose_sigma);
  declare_parameter("zero_velocity_sigma", parameters_.zero_velocity_sigma);
  declare_parameter("fallback_pose_sigma", parameters_.fallback_pose_sigma);
  declare_parameter("imu_accel_sigma", parameters_.imu_accel_sigma);
  declare_parameter("imu_gyro_sigma", parameters_.imu_gyro_sigma);
  declare_parameter("imu_bias_accel_sigma", parameters_.imu_bias_accel_sigma);
  declare_parameter("imu_bias_gyro_sigma", parameters_.imu_bias_gyro_sigma);
  declare_parameter("mag_noise_sigma", parameters_.mag_noise_sigma);
  declare_parameter("mag_field_norm_min", parameters_.mag_field_norm_min);
  declare_parameter("mag_field_norm_max", parameters_.mag_field_norm_max);
  declare_parameter("robust_kernel_scale", parameters_.robust_kernel_scale);
  declare_parameter("queue_size", parameters_.queue_size);
  declare_parameter("reorder_window", parameters_.reorder_window);
  declare_parameter("clock_offset_tolerance", parameters_.clock_offset_tolerance);
  declare_parameter("diagnostics_publish_rate", parameters_.diagnostics_publish_rate);
  declare_parameter("publish_tf", parameters_.publish_tf);
  declare_parameter("lio_lidar_to_base_xyz", parameters_.lio_lidar_to_base_xyz);
  declare_parameter("lio_lidar_to_base_rpy", parameters_.lio_lidar_to_base_rpy);
}

bool StateFusionBackendNode::load_parameters()
{
  parameters_.wheel_topic = get_parameter("wheel_odom_topic").as_string();
  parameters_.imu_topic = get_parameter("imu_topic").as_string();
  parameters_.mag_topic = get_parameter("mag_topic").as_string();
  parameters_.lio_topic = get_parameter("fastlio_topic").as_string();
  parameters_.odom_topic = get_parameter("fused_odom_topic").as_string();
  parameters_.diagnostics_topic = get_parameter("diagnostics_topic").as_string();
  parameters_.odom_frame = get_parameter("odom_frame").as_string();
  parameters_.base_link_frame = get_parameter("base_link_frame").as_string();
  parameters_.wheel_odom_frame = get_parameter("wheel_odom_frame").as_string();
  parameters_.wheel_child_frame = get_parameter("wheel_child_frame").as_string();
  parameters_.lio_world_frame = get_parameter("lio_world_frame").as_string();
  parameters_.lio_child_frame = get_parameter("lio_child_frame").as_string();
  parameters_.fusion_profile = get_parameter("fusion_profile").as_string();
  if (parameters_.fusion_profile.empty()) {
    parameters_.fusion_profile = "default";
  }
  parameters_.use_wheel_odom = get_parameter("use_wheel_odom").as_bool();
  parameters_.use_fast_lio = get_parameter("use_fast_lio").as_bool();
  parameters_.use_lio_health = get_parameter("use_lio_health").as_bool();
  parameters_.use_static_anchor = get_parameter("use_static_anchor").as_bool();
  parameters_.robust_kernel = get_parameter("robust_kernel").as_string();
  parameters_.frequency = get_parameter("frequency").as_double();
  parameters_.sensor_timeout = get_parameter("sensor_timeout").as_double();
  parameters_.max_time_delay = get_parameter("max_time_delay").as_double();
  parameters_.max_future_stamp = get_parameter("max_future_stamp").as_double();
  parameters_.max_pose_jump = get_parameter("max_pose_jump").as_double();
  parameters_.max_pose_jump_yaw = get_parameter("max_pose_jump_yaw").as_double();
  parameters_.lio_degraded_scale = get_parameter("lio_degraded_scale").as_double();
  parameters_.recovery_stable_count = get_parameter("recovery_stable_count").as_int();
  parameters_.lio_nis_degraded_threshold =
    get_parameter("lio_nis_degraded_threshold").as_double();
  parameters_.lio_nis_failed_threshold =
    get_parameter("lio_nis_failed_threshold").as_double();
  parameters_.lio_consistency_degraded_threshold =
    get_parameter("lio_consistency_degraded_threshold").as_double();
  parameters_.lio_consistency_failed_threshold =
    get_parameter("lio_consistency_failed_threshold").as_double();
  parameters_.wheel_consistency_position_sigma =
    get_parameter("wheel_consistency_position_sigma").as_double();
  parameters_.wheel_consistency_yaw_sigma =
    get_parameter("wheel_consistency_yaw_sigma").as_double();
  parameters_.imu_consistency_yaw_sigma =
    get_parameter("imu_consistency_yaw_sigma").as_double();
  parameters_.window_duration = get_parameter("window_duration").as_double();
  parameters_.gravity = get_parameter("gravity").as_double();
  parameters_.use_motioncore_imu = get_parameter("use_motioncore_imu").as_bool();
  parameters_.use_magnetic_heading_factor =
    get_parameter("use_magnetic_heading_factor").as_bool();
  parameters_.mag_calibration_valid = get_parameter("mag_calibration_valid").as_bool();
  parameters_.use_planar_prior = get_parameter("use_planar_prior").as_bool();
  parameters_.planar_z_sigma = get_parameter("planar_z_sigma").as_double();
  parameters_.planar_roll_sigma = get_parameter("planar_roll_sigma").as_double();
  parameters_.planar_pitch_sigma = get_parameter("planar_pitch_sigma").as_double();
  parameters_.planar_reference_z = get_parameter("planar_reference_z").as_double();
  parameters_.planar_reference_roll =
    get_parameter("planar_reference_roll").as_double();
  parameters_.planar_reference_pitch =
    get_parameter("planar_reference_pitch").as_double();
  parameters_.lio_z_sigma_scale = get_parameter("lio_z_sigma_scale").as_double();
  parameters_.wheel_x_sigma = get_parameter("wheel_x_sigma").as_double();
  parameters_.wheel_y_sigma = get_parameter("wheel_y_sigma").as_double();
  parameters_.wheel_yaw_sigma = get_parameter("wheel_yaw_sigma").as_double();
  parameters_.lio_position_sigma = get_parameter("lio_position_sigma").as_double();
  parameters_.lio_rotation_sigma = get_parameter("lio_rotation_sigma").as_double();
  parameters_.stationary_speed_threshold =
    get_parameter("stationary_speed_threshold").as_double();
  parameters_.stationary_yaw_rate_threshold =
    get_parameter("stationary_yaw_rate_threshold").as_double();
  parameters_.static_min_duration = get_parameter("static_min_duration").as_double();
  parameters_.static_gyro_threshold = get_parameter("static_gyro_threshold").as_double();
  parameters_.mag_norm_relative_change_threshold =
    get_parameter("mag_norm_relative_change_threshold").as_double();
  parameters_.stationary_lio_scale =
    get_parameter("stationary_lio_scale").as_double();
  parameters_.stationary_pose_sigma =
    get_parameter("stationary_pose_sigma").as_double();
  parameters_.zero_velocity_sigma = get_parameter("zero_velocity_sigma").as_double();
  parameters_.fallback_pose_sigma = get_parameter("fallback_pose_sigma").as_double();
  parameters_.imu_accel_sigma = get_parameter("imu_accel_sigma").as_double();
  parameters_.imu_gyro_sigma = get_parameter("imu_gyro_sigma").as_double();
  parameters_.imu_bias_accel_sigma = get_parameter("imu_bias_accel_sigma").as_double();
  parameters_.imu_bias_gyro_sigma = get_parameter("imu_bias_gyro_sigma").as_double();
  parameters_.mag_noise_sigma = get_parameter("mag_noise_sigma").as_double();
  parameters_.mag_field_norm_min = get_parameter("mag_field_norm_min").as_double();
  parameters_.mag_field_norm_max = get_parameter("mag_field_norm_max").as_double();
  parameters_.robust_kernel_scale = get_parameter("robust_kernel_scale").as_double();
  parameters_.queue_size = get_parameter("queue_size").as_int();
  parameters_.reorder_window = get_parameter("reorder_window").as_double();
  parameters_.clock_offset_tolerance =
    get_parameter("clock_offset_tolerance").as_double();
  parameters_.diagnostics_publish_rate = get_parameter("diagnostics_publish_rate").as_double();
  parameters_.publish_tf = get_parameter("publish_tf").as_bool();
  parameters_.lio_lidar_to_base_xyz =
    get_parameter("lio_lidar_to_base_xyz").as_double_array();
  parameters_.lio_lidar_to_base_rpy =
    get_parameter("lio_lidar_to_base_rpy").as_double_array();

  const auto profile = parameters_.fusion_profile;
  if (profile == "A") {
    parameters_.use_wheel_odom = true;
    parameters_.use_fast_lio = false;
    parameters_.use_lio_health = false;
    parameters_.use_planar_prior = false;
    parameters_.use_static_anchor = false;
    parameters_.robust_kernel = "gaussian";
  } else if (profile == "B") {
    parameters_.use_wheel_odom = false;
    parameters_.use_fast_lio = true;
    parameters_.use_lio_health = false;
    parameters_.use_planar_prior = false;
    parameters_.use_static_anchor = false;
    parameters_.robust_kernel = "gaussian";
  } else if (profile == "C") {
    parameters_.use_wheel_odom = true;
    parameters_.use_fast_lio = true;
    parameters_.use_lio_health = false;
    parameters_.use_planar_prior = true;
    parameters_.use_static_anchor = false;
    parameters_.robust_kernel = "gaussian";
  } else if (profile == "D") {
    parameters_.use_wheel_odom = true;
    parameters_.use_fast_lio = true;
    parameters_.use_lio_health = false;
    parameters_.use_planar_prior = true;
    parameters_.use_static_anchor = false;
    parameters_.robust_kernel = "huber";
  } else if (profile == "E") {
    parameters_.use_wheel_odom = true;
    parameters_.use_fast_lio = true;
    parameters_.use_lio_health = true;
    parameters_.use_planar_prior = true;
    parameters_.use_static_anchor = false;
    parameters_.robust_kernel = "huber";
  } else if (profile == "F") {
    parameters_.use_wheel_odom = true;
    parameters_.use_fast_lio = true;
    parameters_.use_lio_health = true;
    parameters_.use_planar_prior = true;
    parameters_.use_static_anchor = true;
    parameters_.robust_kernel = "huber";
  } else if (profile == "G") {
    parameters_.use_wheel_odom = true;
    parameters_.use_fast_lio = true;
    parameters_.use_lio_health = true;
    parameters_.use_planar_prior = false;
    parameters_.use_static_anchor = true;
    parameters_.robust_kernel = "huber";
  } else if (profile != "custom" && profile != "default") {
    RCLCPP_ERROR(get_logger(), "Unknown fusion_profile '%s'", profile.c_str());
    return false;
  }

  return (parameters_.use_wheel_odom || parameters_.use_fast_lio) &&
         parameters_.frequency > 0.0 &&
         parameters_.window_duration > 0.0 &&
         parameters_.recovery_stable_count > 0 &&
         parameters_.queue_size > 0 &&
         parameters_.reorder_window >= 0.0 &&
         parameters_.clock_offset_tolerance >= 0.0 &&
         parameters_.lio_lidar_to_base_xyz.size() == 3U &&
         parameters_.lio_lidar_to_base_rpy.size() == 3U;
}

void StateFusionBackendNode::create_entities()
{
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
    parameters_.odom_topic, rclcpp::QoS(10).reliable());
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    parameters_.diagnostics_topic, rclcpp::QoS(10).reliable());
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

  if (parameters_.use_wheel_odom) {
    wheel_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      parameters_.wheel_topic, rclcpp::QoS(20).reliable(),
      std::bind(&StateFusionBackendNode::handle_wheel, this, std::placeholders::_1));
  }
  if (parameters_.use_lio_health) {
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      parameters_.imu_topic, rclcpp::SensorDataQoS(),
      std::bind(&StateFusionBackendNode::handle_imu, this, std::placeholders::_1));
    mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
      parameters_.mag_topic, rclcpp::SensorDataQoS(),
      std::bind(&StateFusionBackendNode::handle_mag, this, std::placeholders::_1));
  }
  if (parameters_.use_fast_lio) {
    lio_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      parameters_.lio_topic, rclcpp::QoS(20).reliable(),
      std::bind(&StateFusionBackendNode::handle_lio, this, std::placeholders::_1));
  }

  wheel_prediction_timer_ = create_wall_timer(
    std::chrono::duration<double>(
      1.0 / std::max(parameters_.frequency, 1.0)),
    std::bind(&StateFusionBackendNode::publish_wheel_prediction, this));

  diagnostics_timer_ = create_wall_timer(
    std::chrono::duration<double>(
      1.0 / std::max(parameters_.diagnostics_publish_rate, 0.1)),
    [this]() {
      std::lock_guard<std::mutex> lock(mutex_);
      if (last_result_ && last_health_) {
        std::uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
        std::string summary = "fusion backend active";
        switch (last_health_->state) {
          case LioHealthState::DEGRADED:
          case LioHealthState::RECOVERING:
            level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            summary = last_health_->reason;
            break;
          case LioHealthState::FAILED:
            level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            summary = last_health_->reason;
            break;
          case LioHealthState::HEALTHY:
            break;
        }
        publish_diagnostics(
          this->get_clock()->now(), *last_result_, *last_health_,
          summary, level);
      }
    });
}

void StateFusionBackendNode::destroy_entities()
{
  wheel_prediction_timer_.reset();
  diagnostics_timer_.reset();
  wheel_sub_.reset();
  imu_sub_.reset();
  mag_sub_.reset();
  lio_sub_.reset();
  tf_broadcaster_.reset();
  odom_pub_.reset();
  diagnostics_pub_.reset();
}

StateFusionBackendNode::CallbackReturn StateFusionBackendNode::on_configure(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!load_parameters()) {
    RCLCPP_ERROR(get_logger(), "Invalid state_fusion_backend parameters");
    return CallbackReturn::FAILURE;
  }
  if (parameters_.use_motioncore_imu || parameters_.use_magnetic_heading_factor) {
    RCLCPP_WARN(
      get_logger(),
      "use_motioncore_imu/use_magnetic_heading_factor are deprecated in v1; "
      "MotionCore IMU and MAG are health-only and will not add localization factors");
  }

  FusionParameters backend_parameters;
  backend_parameters.window_duration = parameters_.window_duration;
  backend_parameters.gravity = parameters_.gravity;
  backend_parameters.use_wheel_odom = parameters_.use_wheel_odom;
  backend_parameters.use_fast_lio = parameters_.use_fast_lio;
  backend_parameters.use_lio_health = parameters_.use_lio_health;
  backend_parameters.use_static_anchor = parameters_.use_static_anchor;
  // IMU/MAG are health-only in v1. Keep the legacy parameters declared and
  // loaded so old launch files remain parseable, but never pass them as
  // switches that can add direct localization factors.
  backend_parameters.use_imu = false;
  backend_parameters.use_mag = false;
  backend_parameters.mag_calibration_valid = false;
  backend_parameters.use_planar_prior = parameters_.use_planar_prior;
  backend_parameters.planar_z_sigma = parameters_.planar_z_sigma;
  backend_parameters.planar_roll_sigma = parameters_.planar_roll_sigma;
  backend_parameters.planar_pitch_sigma = parameters_.planar_pitch_sigma;
  backend_parameters.planar_reference_z = parameters_.planar_reference_z;
  backend_parameters.planar_reference_roll = parameters_.planar_reference_roll;
  backend_parameters.planar_reference_pitch = parameters_.planar_reference_pitch;
  backend_parameters.lio_z_sigma_scale = parameters_.lio_z_sigma_scale;
  backend_parameters.wheel_x_sigma = parameters_.wheel_x_sigma;
  backend_parameters.wheel_y_sigma = parameters_.wheel_y_sigma;
  backend_parameters.wheel_yaw_sigma = parameters_.wheel_yaw_sigma;
  backend_parameters.lio_position_sigma = parameters_.lio_position_sigma;
  backend_parameters.lio_rotation_sigma = parameters_.lio_rotation_sigma;
  backend_parameters.stationary_speed_threshold = parameters_.stationary_speed_threshold;
  backend_parameters.stationary_yaw_rate_threshold =
    parameters_.stationary_yaw_rate_threshold;
  backend_parameters.zero_velocity_sigma = parameters_.zero_velocity_sigma;
  backend_parameters.fallback_pose_sigma = parameters_.fallback_pose_sigma;
  backend_parameters.stationary_lio_scale = parameters_.stationary_lio_scale;
  backend_parameters.stationary_pose_sigma = parameters_.stationary_pose_sigma;
  backend_parameters.imu_accel_sigma = parameters_.imu_accel_sigma;
  backend_parameters.imu_gyro_sigma = parameters_.imu_gyro_sigma;
  backend_parameters.imu_bias_accel_sigma = parameters_.imu_bias_accel_sigma;
  backend_parameters.imu_bias_gyro_sigma = parameters_.imu_bias_gyro_sigma;
  backend_parameters.mag_noise_sigma = parameters_.mag_noise_sigma;
  backend_parameters.mag_field_norm_min = parameters_.mag_field_norm_min;
  backend_parameters.mag_field_norm_max = parameters_.mag_field_norm_max;
  backend_parameters.robust_kernel_scale = parameters_.robust_kernel_scale;
  backend_parameters.robust_kernel = parameters_.robust_kernel;
  const auto & xyz = parameters_.lio_lidar_to_base_xyz;
  const auto & rpy = parameters_.lio_lidar_to_base_rpy;
  backend_parameters.lio_lidar_to_base = gtsam::Pose3(
    // RzRyRx takes [roll, pitch, yaw] despite applying Z, Y, X rotations.
    gtsam::Rot3::RzRyRx(rpy[0], rpy[1], rpy[2]),
    gtsam::Point3(xyz[0], xyz[1], xyz[2]));

  backend_ = std::make_unique<FusionBackend>(backend_parameters);
  health_monitor_ = std::make_unique<LioHealthMonitor>(
    parameters_.max_pose_jump,
    parameters_.max_pose_jump_yaw,
    parameters_.sensor_timeout,
    parameters_.lio_degraded_scale,
    parameters_.recovery_stable_count,
    parameters_.lio_nis_degraded_threshold,
    parameters_.lio_nis_failed_threshold,
    parameters_.lio_consistency_degraded_threshold,
    parameters_.lio_consistency_failed_threshold,
    parameters_.wheel_consistency_position_sigma,
    parameters_.wheel_consistency_yaw_sigma,
    parameters_.imu_consistency_yaw_sigma,
    parameters_.static_min_duration,
    parameters_.static_gyro_threshold,
    parameters_.stationary_speed_threshold,
    parameters_.stationary_yaw_rate_threshold,
    parameters_.mag_norm_relative_change_threshold);
  // Give the first FAST-LIO sample a bounded startup grace period. The
  // watchdog still uses steady receive time, while measurement timestamps
  // remain the source of state-event and output time.
  last_lio_receive_time_ = std::chrono::steady_clock::now();
  create_entities();
  RCLCPP_INFO(
    get_logger(),
    "Configured topics: wheel=%s imu=%s mag=%s lio=%s output=%s; "
    "profile=%s wheel=%s lio=%s health=%s kernel=%s; "
    "FAST-LIO lidar->base extrinsic xyz=[%.6f %.6f %.6f] "
      "rpy=[%.6f %.6f %.6f]",
    parameters_.wheel_topic.c_str(),
    parameters_.imu_topic.c_str(),
    parameters_.mag_topic.c_str(),
    parameters_.lio_topic.c_str(),
    parameters_.odom_topic.c_str(),
    parameters_.fusion_profile.c_str(),
    parameters_.use_wheel_odom ? "true" : "false",
    parameters_.use_fast_lio ? "true" : "false",
    parameters_.use_lio_health ? "true" : "false",
    parameters_.robust_kernel.c_str(),
    xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]);
  return CallbackReturn::SUCCESS;
}

StateFusionBackendNode::CallbackReturn StateFusionBackendNode::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!odom_pub_ || !diagnostics_pub_) {
    return CallbackReturn::FAILURE;
  }
  odom_pub_->on_activate();
  diagnostics_pub_->on_activate();
  RCLCPP_INFO(get_logger(), "state_fusion_backend activated");
  return CallbackReturn::SUCCESS;
}

StateFusionBackendNode::CallbackReturn StateFusionBackendNode::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (odom_pub_) {
    odom_pub_->on_deactivate();
  }
  if (diagnostics_pub_) {
    diagnostics_pub_->on_deactivate();
  }
  RCLCPP_INFO(get_logger(), "state_fusion_backend deactivated");
  return CallbackReturn::SUCCESS;
}

StateFusionBackendNode::CallbackReturn StateFusionBackendNode::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  std::lock_guard<std::mutex> lock(mutex_);
  destroy_entities();
  backend_.reset();
  health_monitor_.reset();
  latest_wheel_.reset();
  wheel_buffer_.clear();
  lio_buffer_.clear();
  latest_mag_.reset();
  imu_buffer_.clear();
  last_processed_imu_stamp_.reset();
  last_processed_wheel_stamp_.reset();
  last_processed_lio_stamp_.reset();
  last_processed_event_stamp_.reset();
  last_received_lio_stamp_.reset();
  last_lio_message_stamp_.reset();
  last_lio_receive_time_.reset();
  last_result_.reset();
  last_health_.reset();
  last_logged_lio_state_.reset();
  last_output_stamp_ns_ = 0;
  return CallbackReturn::SUCCESS;
}

StateFusionBackendNode::CallbackReturn StateFusionBackendNode::on_shutdown(
  const rclcpp_lifecycle::State &)
{
  return on_cleanup(get_current_state());
}

StateFusionBackendNode::CallbackReturn StateFusionBackendNode::on_error(
  const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturn::SUCCESS;
}

void StateFusionBackendNode::handle_wheel(
  const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp);
  if (stamp.nanoseconds() <= 0) {
    return;
  }
  const double now_seconds = get_clock()->now().seconds();
  const double future_age = stamp.seconds() - now_seconds;
  if (std::isfinite(future_age) && future_age > parameters_.max_future_stamp) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Ignoring future-dated wheel odometry: stamp=%.9f now=%.9f "
      "ahead=%.3f s tolerance=%.3f s",
      stamp.seconds(), now_seconds, future_age, parameters_.max_future_stamp);
    return;
  }
  if (
    !msg->header.frame_id.empty() &&
    msg->header.frame_id != parameters_.wheel_odom_frame)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejecting wheel odometry frame '%s', expected '%s'",
      msg->header.frame_id.c_str(), parameters_.wheel_odom_frame.c_str());
    return;
  }
  if (
    !msg->child_frame_id.empty() &&
    msg->child_frame_id != parameters_.wheel_child_frame)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejecting wheel odometry child frame '%s', expected '%s'",
      msg->child_frame_id.c_str(), parameters_.wheel_child_frame.c_str());
    return;
  }

  const WheelObservation observation{
    stamp.seconds(),
    to_pose(msg->pose.pose),
    to_vector(msg->twist.twist.linear),
    to_vector(msg->twist.twist.angular)};

  std::lock_guard<std::mutex> lock(mutex_);
  if (last_processed_wheel_stamp_ &&
    observation.stamp <= *last_processed_wheel_stamp_)
  {
    return;
  }
  if (!wheel_buffer_.empty() &&
    observation.stamp < wheel_buffer_.front().stamp - parameters_.reorder_window)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Dropping wheel observation outside reorder window: stamp=%.9f",
      observation.stamp);
    return;
  }
  const auto insertion = std::upper_bound(
    wheel_buffer_.begin(), wheel_buffer_.end(), observation.stamp,
    [](double value, const WheelObservation & item) {
      return value < item.stamp;
    });
  wheel_buffer_.insert(insertion, observation);
  latest_wheel_ = wheel_buffer_.back();
  while (wheel_buffer_.size() > static_cast<std::size_t>(parameters_.queue_size)) {
    wheel_buffer_.pop_front();
  }
}

void StateFusionBackendNode::publish_wheel_prediction()
{
  bool allow_wheel_prediction = true;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (
      parameters_.use_fast_lio &&
      parameters_.use_lio_health &&
      last_lio_receive_time_ &&
      health_monitor_)
    {
      const double lio_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - *last_lio_receive_time_).count();
      if (
        lio_age > parameters_.sensor_timeout &&
        health_monitor_->state() != LioHealthState::FAILED)
      {
        last_health_ = health_monitor_->check_timeout(lio_age);
        const auto now = get_clock()->now();
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "FAST-LIO watchdog timeout: receive_age=%.3f s threshold=%.3f s "
          "last_message_stamp=%.9f s node_time=%.9f s",
          lio_age, parameters_.sensor_timeout,
          last_lio_message_stamp_ ? last_lio_message_stamp_->seconds() : 0.0,
          now.seconds());
        log_lio_state_transition(now, *last_health_);
      }
    }

    if (parameters_.use_fast_lio) {
      const bool lio_timed_out =
        !last_lio_receive_time_ ||
        std::chrono::duration<double>(
        std::chrono::steady_clock::now() - *last_lio_receive_time_).count() >
        parameters_.sensor_timeout;
      allow_wheel_prediction = lio_timed_out;
    }
  }

  process_pending();

  std::lock_guard<std::mutex> lock(mutex_);
  if (!allow_wheel_prediction || !lio_buffer_.empty()) {
    return;
  }

  if (
    backend_ == nullptr ||
    !backend_->initialized() ||
    !latest_wheel_ ||
    !last_result_ ||
    !last_health_ ||
    !odom_pub_ ||
    !odom_pub_->is_activated())
  {
    return;
  }

  if (!parameters_.use_wheel_odom) {
    return;
  }

  const auto stamp_ns = static_cast<std::int64_t>(
    std::llround(latest_wheel_->stamp * 1.0e9));
  if (stamp_ns <= last_output_stamp_ns_) {
    return;
  }

  const rclcpp::Time wheel_stamp(stamp_ns, RCL_ROS_TIME);

  const auto predicted_pose = backend_->predicted_pose(&*latest_wheel_);
  if (!predicted_pose) {
    return;
  }

  FusionResult prediction = *last_result_;
  prediction.pose = *predicted_pose;
  prediction.velocity =
    prediction.pose.rotation().rotate(latest_wheel_->linear_velocity);
  prediction.angular_velocity = latest_wheel_->angular_velocity;
  prediction.used_wheel = true;
  prediction.used_lio = false;
  prediction.factors_used.clear();
  prediction.success = true;

  publish_result(wheel_stamp, prediction, *last_health_);
  last_output_stamp_ns_ = stamp_ns;
}

void StateFusionBackendNode::process_pending()
{
  constexpr std::size_t kMaxUpdatesPerTick = 64U;

  for (std::size_t update_count = 0; update_count < kMaxUpdatesPerTick; ++update_count) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (
      backend_ == nullptr ||
      health_monitor_ == nullptr ||
      !odom_pub_ ||
      !odom_pub_->is_activated())
    {
      return;
    }

    const bool lio_timed_out =
      !last_lio_receive_time_ ||
      std::chrono::duration<double>(
      std::chrono::steady_clock::now() - *last_lio_receive_time_).count() >
      parameters_.sensor_timeout;

    std::optional<WheelObservation> wheel;
    std::optional<LioObservation> lio;
    bool lio_event = false;

    if (!parameters_.use_fast_lio) {
      if (wheel_buffer_.empty()) {
        return;
      }
      wheel = wheel_buffer_.front();
    } else if (!parameters_.use_wheel_odom) {
      if (lio_buffer_.empty()) {
        return;
      }
      lio = lio_buffer_.front();
      lio_event = true;
    } else if (!lio_buffer_.empty()) {
      // Profiles C-G use FAST-LIO timestamps as the primary state clock.
      // Wheel data is attached causally at or before the LIO measurement.
      lio = lio_buffer_.front();
      lio_event = true;

      for (auto iterator = wheel_buffer_.rbegin();
        iterator != wheel_buffer_.rend(); ++iterator)
      {
        if (
          iterator->stamp <= lio->stamp &&
          lio->stamp - iterator->stamp <= parameters_.max_time_delay)
        {
          wheel = *iterator;
          break;
        }
      }
    } else {
      // During normal operation, wait for the LIO clock. Once the receive
      // watchdog expires, advance on wheel timestamps to preserve odometry
      // continuity during a Raspberry Pi/network dropout.
      if (wheel_buffer_.empty() || !lio_timed_out) {
        return;
      }
      wheel = wheel_buffer_.front();
    }

    const double event_stamp = lio_event ? lio->stamp : wheel->stamp;
    if (
      !std::isfinite(event_stamp) ||
      event_stamp <= 0.0 ||
      (last_processed_event_stamp_ &&
      event_stamp <= *last_processed_event_stamp_))
    {
      if (lio_event) {
        lio_buffer_.pop_front();
        last_processed_lio_stamp_ = lio->stamp;
      } else {
        wheel_buffer_.pop_front();
        last_processed_wheel_stamp_ = wheel->stamp;
      }
      continue;
    }

    std::vector<ImuSample> imu_samples;
    if (parameters_.use_lio_health) {
      for (const auto & sample : imu_buffer_) {
        if (
          sample.stamp <= event_stamp &&
          (!last_processed_imu_stamp_ || sample.stamp > *last_processed_imu_stamp_))
        {
          imu_samples.push_back(sample);
        }
      }
    }

    std::optional<MagObservation> mag;
    if (
      parameters_.use_lio_health &&
      latest_mag_ &&
      latest_mag_->stamp <= event_stamp &&
      event_stamp - latest_mag_->stamp <= parameters_.max_time_delay)
    {
      mag = latest_mag_;
    }

    LioHealthResult health = last_health_.value_or(LioHealthResult{});
    if (lio_event && parameters_.use_lio_health) {
      LioQualityContext context;
      context.wheel_valid = wheel.has_value() && backend_->initialized();
      if (context.wheel_valid) {
        const auto wheel_delta = backend_->wheel_delta(&*wheel);
        if (wheel_delta) {
          context.wheel_delta = *wheel_delta;
        } else {
          context.wheel_valid = false;
        }
      }

      if (backend_->initialized()) {
        const auto predicted_pose = backend_->predicted_lio_pose(
          wheel ? &*wheel : nullptr);
        if (predicted_pose) {
          context.prediction_valid = true;
          context.predicted_pose = *predicted_pose;
        }
      }

      context.imu_valid = !imu_samples.empty();
      if (context.imu_valid) {
        context.imu_gyro_norm = imu_samples.back().angular_velocity.norm();
        double previous_stamp = imu_samples.front().stamp;
        double previous_gyro_z = imu_samples.front().angular_velocity.z();
        for (std::size_t index = 1; index < imu_samples.size(); ++index) {
          const auto & sample = imu_samples[index];
          const double dt = sample.stamp - previous_stamp;
          if (std::isfinite(dt) && dt > 0.0) {
            context.imu_yaw_delta += previous_gyro_z * dt;
          }
          previous_stamp = sample.stamp;
          previous_gyro_z = sample.angular_velocity.z();
        }
        const double tail_dt = event_stamp - previous_stamp;
        if (std::isfinite(tail_dt) && tail_dt > 0.0) {
          context.imu_yaw_delta += previous_gyro_z * tail_dt;
        }
      }

      context.static_candidate =
        wheel &&
        std::hypot(
        wheel->linear_velocity.x(), wheel->linear_velocity.y()) <=
        parameters_.stationary_speed_threshold &&
        std::abs(wheel->angular_velocity.z()) <=
        parameters_.stationary_yaw_rate_threshold;
      if (mag) {
        context.mag_valid = true;
        context.mag_field_norm = mag->field.norm();
      }
      context.covariance_valid = lio->covariance_valid;
      context.covariance = lio->covariance;
      context.fallback_position_sigma = parameters_.lio_position_sigma;
      context.fallback_rotation_sigma = parameters_.lio_rotation_sigma;
      context.lio_z_sigma_scale = parameters_.lio_z_sigma_scale;

      health = health_monitor_->observe(
        lio->stamp, backend_->lio_pose_for_health(lio->pose), context);
      log_lio_state_transition(
        rclcpp::Time(
        static_cast<std::int64_t>(std::llround(lio->stamp * 1.0e9)),
        RCL_ROS_TIME),
        health);
    }

    FusionResult result;
    try {
      result = backend_->update(
        wheel ? &*wheel : nullptr,
        lio ? &*lio : nullptr,
        imu_samples,
        mag ? &*mag : nullptr,
        health);
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(
        get_logger(),
        "Fusion graph update failed at %.9f s: %s; resetting graph",
        event_stamp, exception.what());
      backend_->reset();
      health_monitor_->reset();
      last_result_.reset();
      last_health_.reset();
      last_processed_event_stamp_ = event_stamp;
      if (lio_event) {
        lio_buffer_.pop_front();
        last_processed_lio_stamp_ = lio->stamp;
      } else {
        wheel_buffer_.pop_front();
        last_processed_wheel_stamp_ = wheel->stamp;
      }
      continue;
    }

    if (lio_event) {
      lio_buffer_.pop_front();
      last_processed_lio_stamp_ = lio->stamp;
      last_received_lio_stamp_ = lio->stamp;
    }
    if (wheel) {
      while (!wheel_buffer_.empty() &&
        wheel_buffer_.front().stamp <= wheel->stamp)
      {
        wheel_buffer_.pop_front();
      }
      last_processed_wheel_stamp_ = wheel->stamp;
      latest_wheel_ = *wheel;
    }
    if (!imu_samples.empty()) {
      last_processed_imu_stamp_ = imu_samples.back().stamp;
      while (!imu_buffer_.empty() &&
        imu_buffer_.front().stamp <= *last_processed_imu_stamp_)
      {
        imu_buffer_.pop_front();
      }
    }

    last_processed_event_stamp_ = event_stamp;
    last_result_ = result;
    last_health_ = health;

    const auto stamp_ns = static_cast<std::int64_t>(
      std::llround(event_stamp * 1.0e9));
    if (result.success && stamp_ns > last_output_stamp_ns_) {
      publish_result(
        rclcpp::Time(stamp_ns, RCL_ROS_TIME), result, health);
      last_output_stamp_ns_ = stamp_ns;
    }
  }
}

void StateFusionBackendNode::handle_imu(
  const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp);
  if (stamp.nanoseconds() <= 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  imu_buffer_.push_back(
    ImuSample{
      stamp.seconds(),
      to_vector(msg->linear_acceleration),
      to_vector(msg->angular_velocity)});
  while (imu_buffer_.size() > static_cast<std::size_t>(parameters_.queue_size)) {
    imu_buffer_.pop_front();
  }
}

void StateFusionBackendNode::handle_mag(
  const sensor_msgs::msg::MagneticField::ConstSharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp);
  if (stamp.nanoseconds() <= 0) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  latest_mag_ = MagObservation{stamp.seconds(), to_vector(msg->magnetic_field)};
}

void StateFusionBackendNode::handle_lio(
  const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  const rclcpp::Time stamp(msg->header.stamp);
  if (stamp.nanoseconds() <= 0 || backend_ == nullptr || health_monitor_ == nullptr) {
    return;
  }

  if (
    !msg->header.frame_id.empty() &&
    msg->header.frame_id != parameters_.lio_world_frame)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejecting FAST-LIO frame '%s', expected '%s'",
      msg->header.frame_id.c_str(), parameters_.lio_world_frame.c_str());
    return;
  }
  if (
    !msg->child_frame_id.empty() &&
    msg->child_frame_id != parameters_.lio_child_frame)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejecting FAST-LIO child frame '%s', expected '%s'",
      msg->child_frame_id.c_str(), parameters_.lio_child_frame.c_str());
    return;
  }

  bool covariance_valid = false;
  const gtsam::Matrix6 covariance = to_covariance(msg->pose.covariance, &covariance_valid);
  const LioObservation observation{
    stamp.seconds(), to_pose(msg->pose.pose), covariance, covariance_valid};

  std::lock_guard<std::mutex> lock(mutex_);
  if (last_processed_lio_stamp_ &&
    observation.stamp <= *last_processed_lio_stamp_)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Ignoring non-increasing FAST-LIO timestamp: %.9f",
      stamp.seconds());
    return;
  }
  if (!lio_buffer_.empty() &&
    observation.stamp < lio_buffer_.front().stamp - parameters_.reorder_window)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Dropping FAST-LIO observation outside reorder window: stamp=%.9f",
      observation.stamp);
    return;
  }

  last_lio_receive_time_ = std::chrono::steady_clock::now();
  last_lio_message_stamp_ = stamp;
  last_received_lio_stamp_ = observation.stamp;
  const auto insertion = std::upper_bound(
    lio_buffer_.begin(), lio_buffer_.end(), observation.stamp,
    [](double value, const LioObservation & item) {
      return value < item.stamp;
    });
  lio_buffer_.insert(insertion, observation);
  while (lio_buffer_.size() > static_cast<std::size_t>(parameters_.queue_size)) {
    lio_buffer_.pop_front();
  }
}

void StateFusionBackendNode::log_lio_state_transition(
  const rclcpp::Time & stamp,
  const LioHealthResult & health)
{
  if (last_logged_lio_state_ && *last_logged_lio_state_ == health.state) {
    return;
  }

  const char * state = LioHealthMonitor::state_name(health.state);
  const char * previous_state = last_logged_lio_state_ ?
    LioHealthMonitor::state_name(*last_logged_lio_state_) : "UNINITIALIZED";

  switch (health.state) {
    case LioHealthState::HEALTHY:
      RCLCPP_INFO(
        get_logger(),
        "LIO health state transition at %.9f s: %s -> %s; "
        "reason=%s accept_factor=%s requires_alignment=%s static=%s "
        "covariance_scale=%.3f nis=%.3f innovation_norm=%.3f "
        "wheel_consistency=%.3f imu_consistency=%.3f mag_consistency=%.3f",
        stamp.seconds(), previous_state, state, health.reason.c_str(),
        health.accept_factor ? "true" : "false",
        health.requires_alignment ? "true" : "false",
        health.static_detected ? "true" : "false",
        health.covariance_scale, health.nis, health.innovation_norm,
        health.wheel_consistency, health.imu_consistency,
        health.mag_consistency);
      break;
    case LioHealthState::DEGRADED:
    case LioHealthState::RECOVERING:
      RCLCPP_WARN(
        get_logger(),
        "LIO health state transition at %.9f s: %s -> %s; "
        "reason=%s accept_factor=%s requires_alignment=%s static=%s "
        "covariance_scale=%.3f nis=%.3f innovation_norm=%.3f "
        "wheel_consistency=%.3f imu_consistency=%.3f mag_consistency=%.3f",
        stamp.seconds(), previous_state, state, health.reason.c_str(),
        health.accept_factor ? "true" : "false",
        health.requires_alignment ? "true" : "false",
        health.static_detected ? "true" : "false",
        health.covariance_scale, health.nis, health.innovation_norm,
        health.wheel_consistency, health.imu_consistency,
        health.mag_consistency);
      break;
    case LioHealthState::FAILED:
      RCLCPP_ERROR(
        get_logger(),
        "LIO health state transition at %.9f s: %s -> %s; "
        "reason=%s accept_factor=%s requires_alignment=%s static=%s "
        "covariance_scale=%.3f nis=%.3f innovation_norm=%.3f "
        "wheel_consistency=%.3f imu_consistency=%.3f mag_consistency=%.3f",
        stamp.seconds(), previous_state, state, health.reason.c_str(),
        health.accept_factor ? "true" : "false",
        health.requires_alignment ? "true" : "false",
        health.static_detected ? "true" : "false",
        health.covariance_scale, health.nis, health.innovation_norm,
        health.wheel_consistency, health.imu_consistency,
        health.mag_consistency);
      break;
  }

  last_logged_lio_state_ = health.state;
}

void StateFusionBackendNode::publish_result(
  const rclcpp::Time & stamp,
  const FusionResult & result,
  const LioHealthResult & health)
{
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = stamp;
  odom.header.frame_id = parameters_.odom_frame;
  odom.child_frame_id = parameters_.base_link_frame;
  odom.pose.pose = to_pose_message(result.pose);

  const auto body_velocity = result.pose.rotation().unrotate(result.velocity);
  odom.twist.twist.linear.x = body_velocity.x();
  odom.twist.twist.linear.y = body_velocity.y();
  odom.twist.twist.linear.z = body_velocity.z();
  const auto body_angular_velocity =
    result.pose.rotation().unrotate(result.angular_velocity);
  odom.twist.twist.angular.x = body_angular_velocity.x();
  odom.twist.twist.angular.y = body_angular_velocity.y();
  odom.twist.twist.angular.z = body_angular_velocity.z();
  // nav_msgs/Odometry publishes variances, while health covariance_scale is
  // applied to the standard deviation used by the factor noise model.
  const double position_sigma =
    parameters_.lio_position_sigma * health.covariance_scale;
  const double z_sigma = position_sigma * parameters_.lio_z_sigma_scale;
  const double rotation_sigma =
    parameters_.lio_rotation_sigma * health.covariance_scale;
  odom.pose.covariance[0] = position_sigma * position_sigma;
  odom.pose.covariance[7] = position_sigma * position_sigma;
  odom.pose.covariance[14] = z_sigma * z_sigma;
  odom.pose.covariance[21] = rotation_sigma * rotation_sigma;
  odom.pose.covariance[28] = rotation_sigma * rotation_sigma;
  odom.pose.covariance[35] = rotation_sigma * rotation_sigma;

  if (odom_pub_ && odom_pub_->is_activated()) {
    odom_pub_->publish(odom);
  }

  if (parameters_.publish_tf && tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = odom.child_frame_id;
    transform.transform.translation.x = result.pose.translation().x();
    transform.transform.translation.y = result.pose.translation().y();
    transform.transform.translation.z = result.pose.translation().z();
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }
}

void StateFusionBackendNode::publish_diagnostics(
  const rclcpp::Time & stamp,
  const FusionResult & result,
  const LioHealthResult & health,
  const std::string & summary,
  std::uint8_t level)
{
  if (!diagnostics_pub_ || !diagnostics_pub_->is_activated()) {
    return;
  }
  const auto stamp_ns = stamp.nanoseconds();
  const auto min_interval_ns = static_cast<std::int64_t>(
    1.0e9 / std::max(parameters_.diagnostics_publish_rate, 0.1));
  if (
    last_diagnostics_stamp_ns_ != 0 &&
    stamp_ns - last_diagnostics_stamp_ns_ < min_interval_ns)
  {
    return;
  }
  last_diagnostics_stamp_ns_ = stamp_ns;

  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = stamp;
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "state_fusion_backend";
  status.hardware_id = "FusionLynx";
  status.level = level;
  status.message = summary;

  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "lio_state";
  kv.value = LioHealthMonitor::state_name(health.state);
  status.values.push_back(kv);
  kv.key = "lio_covariance_scale";
  kv.value = std::to_string(health.covariance_scale);
  status.values.push_back(kv);
  kv.key = "lio_translation_jump";
  kv.value = std::to_string(health.translation_jump);
  status.values.push_back(kv);
  kv.key = "lio_yaw_jump";
  kv.value = std::to_string(health.yaw_jump);
  status.values.push_back(kv);
  kv.key = "lio_innovation_norm";
  kv.value = std::to_string(health.innovation_norm);
  status.values.push_back(kv);
  kv.key = "lio_nis";
  kv.value = std::to_string(health.nis);
  status.values.push_back(kv);
  kv.key = "lio_wheel_consistency";
  kv.value = std::to_string(health.wheel_consistency);
  status.values.push_back(kv);
  kv.key = "lio_imu_consistency";
  kv.value = std::to_string(health.imu_consistency);
  status.values.push_back(kv);
  kv.key = "lio_mag_consistency";
  kv.value = std::to_string(health.mag_consistency);
  status.values.push_back(kv);
  kv.key = "lio_confidence";
  kv.value = std::to_string(health.confidence);
  status.values.push_back(kv);
  kv.key = "lio_covariance_source";
  kv.value = health.used_measurement_covariance ? "fast_lio" : "fallback";
  status.values.push_back(kv);
  kv.key = "used_wheel";
  kv.value = result.used_wheel ? "true" : "false";
  status.values.push_back(kv);
  kv.key = "used_imu";
  kv.value = result.used_imu ? "true" : "false";
  status.values.push_back(kv);
  kv.key = "used_mag";
  kv.value = result.used_mag ? "true" : "false";
  status.values.push_back(kv);
  kv.key = "stationary";
  kv.value = result.stationary ? "true" : "false";
  status.values.push_back(kv);
  kv.key = "optimization_error";
  kv.value = std::to_string(result.optimization_error);
  status.values.push_back(kv);
  array.status.push_back(status);
  diagnostics_pub_->publish(array);
}

gtsam::Pose3 StateFusionBackendNode::to_pose(const geometry_msgs::msg::Pose & pose)
{
  const gtsam::Rot3 rotation = gtsam::Rot3::Quaternion(
    pose.orientation.w,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z);
  return gtsam::Pose3(
    rotation,
    gtsam::Point3(pose.position.x, pose.position.y, pose.position.z));
}

gtsam::Matrix6 StateFusionBackendNode::to_covariance(
  const std::array<double, 36> & covariance,
  bool * valid)
{
  // Keep the incoming ROS covariance layout unchanged. The factor and NIS
  // layers explicitly remap it to GTSAM's Pose3 tangent layout.
  gtsam::Matrix6 result = gtsam::Matrix6::Zero();
  bool diagonal_valid = true;
  for (int row = 0; row < 6; ++row) {
    for (int column = 0; column < 6; ++column) {
      result(row, column) = covariance[static_cast<std::size_t>(row * 6 + column)];
    }
    const double diagonal = result(row, row);
    diagonal_valid = diagonal_valid &&
      std::isfinite(diagonal) && diagonal > 1.0e-12 && diagonal < 1.0e6;
  }
  if (valid != nullptr) {
    *valid = diagonal_valid;
  }
  return result;
}

geometry_msgs::msg::Pose StateFusionBackendNode::to_pose_message(const gtsam::Pose3 & pose)
{
  geometry_msgs::msg::Pose result;
  const auto quaternion = pose.rotation().toQuaternion();
  result.position.x = pose.translation().x();
  result.position.y = pose.translation().y();
  result.position.z = pose.translation().z();
  result.orientation.w = quaternion.w();
  result.orientation.x = quaternion.x();
  result.orientation.y = quaternion.y();
  result.orientation.z = quaternion.z();
  return result;
}

gtsam::Vector3 StateFusionBackendNode::to_vector(const geometry_msgs::msg::Vector3 & value)
{
  return gtsam::Vector3(value.x, value.y, value.z);
}

}  // namespace state_fusion_backend
