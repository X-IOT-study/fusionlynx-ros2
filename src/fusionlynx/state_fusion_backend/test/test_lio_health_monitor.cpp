#include <gtest/gtest.h>

#include <gtsam/geometry/Pose3.h>

#include "state_fusion_backend/lio_health_monitor.hpp"

namespace state_fusion_backend
{
namespace
{

LioHealthMonitor make_monitor()
{
  return LioHealthMonitor(
    1.0, 0.5, 0.5, 10.0, 3,
    16.8, 30.0, 3.0, 6.0,
    0.10, 0.08, 0.10,
    0.5, 0.03, 0.02, 0.03, 0.20);
}

LioQualityContext static_context()
{
  LioQualityContext context;
  context.prediction_valid = true;
  context.predicted_pose = gtsam::Pose3();
  context.wheel_valid = true;
  context.wheel_delta = gtsam::Pose3();
  context.imu_valid = true;
  context.imu_gyro_norm = 0.001;
  context.imu_yaw_delta = 0.0;
  context.static_candidate = true;
  return context;
}

}  // namespace

TEST(LioHealthMonitor, RequiresContinuousStaticEvidence)
{
  auto monitor = make_monitor();
  const auto context = static_context();

  EXPECT_FALSE(monitor.observe(1.0, gtsam::Pose3(), context).static_detected);
  EXPECT_FALSE(monitor.observe(1.2, gtsam::Pose3(), context).static_detected);
  EXPECT_TRUE(monitor.observe(1.6, gtsam::Pose3(), context).static_detected);
}

TEST(LioHealthMonitor, KeepsConsistentSmallMotion)
{
  auto monitor = make_monitor();
  const gtsam::Pose3 first;
  monitor.observe(1.0, first, LioQualityContext{});

  LioQualityContext context;
  context.prediction_valid = true;
  context.predicted_pose = gtsam::Pose3(
    gtsam::Rot3(), gtsam::Point3(0.01, 0.0, 0.0));
  context.wheel_valid = true;
  context.wheel_delta = context.predicted_pose;
  context.imu_valid = true;
  context.imu_yaw_delta = 0.0;
  context.imu_gyro_norm = 0.001;

  const auto result = monitor.observe(
    1.1,
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.01, 0.0, 0.0)),
    context);
  EXPECT_TRUE(result.accept_factor);
  EXPECT_EQ(result.state, LioHealthState::HEALTHY);
}

TEST(LioHealthMonitor, FloorsOverconfidentLioCovariance)
{
  auto monitor = make_monitor();
  monitor.observe(1.0, gtsam::Pose3(), LioQualityContext{});

  LioQualityContext context;
  context.prediction_valid = true;
  context.predicted_pose = gtsam::Pose3();
  context.covariance_valid = true;
  context.covariance = gtsam::Matrix6::Identity() * 1.0e-6;
  context.fallback_position_sigma = 0.05;
  context.fallback_rotation_sigma = 0.03;

  const auto result = monitor.observe(
    1.1,
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.01, 0.0, 0.0)),
    context);
  EXPECT_EQ(result.state, LioHealthState::HEALTHY);
  EXPECT_TRUE(result.accept_factor);
}

TEST(LioHealthMonitor, RecoversAfterRepeatedInnovationFailure)
{
  auto monitor = make_monitor();
  monitor.observe(1.0, gtsam::Pose3(), LioQualityContext{});

  LioQualityContext bad;
  bad.prediction_valid = true;
  bad.predicted_pose = gtsam::Pose3();
  const gtsam::Pose3 bad_pose(
    gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0));
  EXPECT_TRUE(monitor.observe(1.1, bad_pose, bad).accept_factor);
  const auto failed = monitor.observe(1.2, bad_pose, bad);
  EXPECT_FALSE(failed.accept_factor);
  EXPECT_EQ(failed.state, LioHealthState::FAILED);

  LioQualityContext recovery;
  recovery.wheel_valid = true;
  recovery.wheel_delta = gtsam::Pose3();
  recovery.imu_valid = true;
  recovery.imu_gyro_norm = 0.001;
  recovery.imu_yaw_delta = 0.0;
  const auto recovered = monitor.observe(1.3, bad_pose, recovery);
  EXPECT_TRUE(recovered.accept_factor);
  EXPECT_EQ(recovered.state, LioHealthState::RECOVERING);
  EXPECT_TRUE(recovered.requires_alignment);
}

TEST(LioHealthMonitor, UsesMagneticDisturbanceOnlyForLioHealth)
{
  auto monitor = make_monitor();
  LioQualityContext context;
  context.mag_valid = true;
  context.mag_field_norm = 50.0e-6;

  monitor.observe(1.0, gtsam::Pose3(), context);
  monitor.observe(1.1, gtsam::Pose3(), context);

  context.mag_field_norm = 65.0e-6;
  const auto result = monitor.observe(1.2, gtsam::Pose3(), context);

  EXPECT_GT(result.mag_consistency, 1.0);
  EXPECT_GT(result.covariance_scale, 1.0);
  EXPECT_TRUE(result.accept_factor);
}

TEST(LioHealthMonitor, TransitionsToFailedOnTimeout)
{
  auto monitor = make_monitor();
  monitor.observe(1.0, gtsam::Pose3(), LioQualityContext{});

  const auto result = monitor.check_timeout(0.6);
  EXPECT_EQ(result.state, LioHealthState::FAILED);
  EXPECT_FALSE(result.accept_factor);
  EXPECT_EQ(monitor.state(), LioHealthState::FAILED);
}

TEST(LioHealthMonitor, DoesNotUseSensorTimestampDomainForTimeout)
{
  auto monitor = make_monitor();
  monitor.observe(1000.0, gtsam::Pose3(), LioQualityContext{});

  const auto result = monitor.check_timeout(0.1);
  EXPECT_EQ(result.state, LioHealthState::HEALTHY);
  EXPECT_TRUE(result.accept_factor);
}

}  // namespace state_fusion_backend
