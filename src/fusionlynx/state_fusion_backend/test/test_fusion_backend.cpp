#include <gtest/gtest.h>

#include <algorithm>

#include <gtsam/geometry/Pose3.h>

#include "state_fusion_backend/factor_graph_builder.hpp"
#include "state_fusion_backend/fusion_backend.hpp"

namespace state_fusion_backend
{
namespace
{

FusionParameters test_parameters()
{
  FusionParameters parameters;
  parameters.window_duration = 5.0;
  parameters.use_planar_prior = false;
  parameters.lio_lidar_to_base = gtsam::Pose3();
  parameters.robust_kernel_scale = 1.0;
  parameters.lio_position_sigma = 0.05;
  parameters.lio_rotation_sigma = 0.03;
  parameters.wheel_x_sigma = 0.02;
  parameters.wheel_y_sigma = 0.02;
  parameters.wheel_yaw_sigma = 0.03;
  return parameters;
}

LioObservation lio_observation(double stamp, double x, double y, double yaw = 0.0)
{
  return LioObservation{
    stamp,
    gtsam::Pose3(
      gtsam::Rot3::Yaw(yaw),
      gtsam::Point3(x, y, 0.0)),
    gtsam::Matrix6::Zero(),
    false};
}

WheelObservation wheel_observation(double stamp, double x, double y, double yaw = 0.0)
{
  return WheelObservation{
    stamp,
    gtsam::Pose3(
      gtsam::Rot3::Yaw(yaw),
      gtsam::Point3(x, y, 0.0)),
    gtsam::Vector3::Zero(),
    gtsam::Vector3::Zero()};
}

bool has_factor(const FusionResult & result, FactorType type)
{
  return std::find(
    result.factors_used.begin(),
    result.factors_used.end(),
    type) != result.factors_used.end();
}

}  // namespace

TEST(FusionBackend, UsesRelativePlanarPriorAgainstLioZDrift)
{
  auto parameters = test_parameters();
  parameters.use_planar_prior = true;
  parameters.planar_z_sigma = 0.05;
  parameters.planar_roll_sigma = 0.05;
  parameters.planar_pitch_sigma = 0.05;
  parameters.lio_z_sigma_scale = 10.0;

  FusionBackend backend(parameters);
  const LioHealthResult healthy;

  const auto first_wheel = wheel_observation(1.0, 0.0, 0.0);
  const auto first_lio = lio_observation(1.0, 0.0, 0.0);
  ASSERT_TRUE(backend.update(&first_wheel, first_lio, {}, nullptr, healthy).success);

  const auto second_wheel = wheel_observation(1.1, 0.1, 0.0);
  const auto second_lio = LioObservation{
    1.1,
    gtsam::Pose3(gtsam::Rot3::Yaw(0.0), gtsam::Point3(0.1, 0.0, 1.0)),
    gtsam::Matrix6::Zero(),
    false};
  ASSERT_TRUE(
    backend.update(&second_wheel, second_lio, {}, nullptr, healthy).success);

  const auto third_wheel = wheel_observation(1.2, 0.2, 0.0);
  const auto third_lio = LioObservation{
    1.2,
    gtsam::Pose3(gtsam::Rot3::Yaw(0.0), gtsam::Point3(0.2, 0.0, 2.0)),
    gtsam::Matrix6::Zero(),
    false};
  const auto result =
    backend.update(&third_wheel, third_lio, {}, nullptr, healthy);
  ASSERT_TRUE(result.success);
  EXPECT_LT(std::abs(result.pose.translation().z()), 0.30);
}

TEST(FusionBackend, UsesConfiguredPlanarReference)
{
  auto parameters = test_parameters();
  parameters.use_planar_prior = true;
  parameters.planar_z_sigma = 0.02;
  parameters.planar_roll_sigma = 0.02;
  parameters.planar_pitch_sigma = 0.02;
  parameters.planar_reference_z = 0.35;

  FusionBackend backend(parameters);
  const LioHealthResult healthy;
  ASSERT_TRUE(
    backend.update(
      nullptr, lio_observation(1.0, 0.0, 0.0), {}, nullptr, healthy).success);

  const auto result = backend.update(
    nullptr, lio_observation(1.1, 0.0, 0.0), {}, nullptr, healthy);
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.pose.translation().z(), 0.35, 0.03);
}

TEST(FusionBackend, PreservesPlanarTranslationAndYaw)
{
  FusionBackend backend(test_parameters());
  const LioHealthResult healthy;

  const auto first_wheel = wheel_observation(1.0, 0.0, 0.0);
  const auto first_lio = lio_observation(1.0, 0.0, 0.0);
  const auto first = backend.update(
    &first_wheel, first_lio,
    {}, nullptr, healthy);
  ASSERT_TRUE(first.success);

  const auto second_wheel = wheel_observation(1.1, 1.0, 0.25, 0.20);
  const auto second_lio = lio_observation(1.1, 1.0, 0.25, 0.20);
  const auto second = backend.update(
    &second_wheel, second_lio,
    {}, nullptr, healthy);
  ASSERT_TRUE(second.success);
  EXPECT_NEAR(second.pose.translation().x(), 1.0, 1.0e-3);
  EXPECT_NEAR(second.pose.translation().y(), 0.25, 1.0e-3);
  EXPECT_NEAR(second.pose.rotation().yaw(), 0.20, 1.0e-3);
}

TEST(FusionBackend, AlignsFirstFrameAfterLidarToBaseConversion)
{
  auto parameters = test_parameters();
  parameters.lio_lidar_to_base = gtsam::Pose3(
    gtsam::Rot3::Yaw(1.5707963267948966),
    gtsam::Point3(1.0, 0.0, 0.0));
  FusionBackend backend(parameters);
  const LioHealthResult healthy;

  auto first_wheel = wheel_observation(1.0, 0.0, 0.0);
  first_wheel.linear_velocity.x() = 0.3;
  first_wheel.angular_velocity.z() = 0.4;
  const auto first = LioObservation{
    1.0,
    gtsam::Pose3(gtsam::Rot3::Yaw(0.0), gtsam::Point3(10.0, 0.0, 0.0)),
    gtsam::Matrix6::Zero(),
    false};
  const auto first_result = backend.update(
    &first_wheel, first, {}, nullptr, healthy);

  ASSERT_TRUE(first_result.success);
  EXPECT_TRUE(first_result.initialized);
  EXPECT_FALSE(first_result.used_wheel);
  EXPECT_FALSE(first_result.used_lio);
  EXPECT_NEAR(first_result.pose.translation().norm(), 0.0, 1.0e-9);
  EXPECT_NEAR(first_result.velocity.x(), 0.3, 1.0e-9);
  EXPECT_NEAR(first_result.angular_velocity.z(), 0.4, 1.0e-9);

  const auto second = LioObservation{
    1.1,
    gtsam::Pose3(gtsam::Rot3::Yaw(0.0), gtsam::Point3(10.0, 1.0, 0.0)),
    gtsam::Matrix6::Zero(),
    false};
  const auto second_result = backend.update(
    nullptr, second, {}, nullptr, healthy);

  ASSERT_TRUE(second_result.success);
  EXPECT_TRUE(second_result.used_lio);
  EXPECT_NEAR(second_result.pose.translation().x(), 1.0, 1.0e-3);
  EXPECT_NEAR(second_result.pose.translation().y(), 0.0, 1.0e-3);
  EXPECT_NEAR(second_result.pose.rotation().yaw(), 0.0, 1.0e-3);
}

TEST(FusionBackend, CarriesWheelAngularVelocityToFusionResult)
{
  FusionBackend backend(test_parameters());
  const LioHealthResult healthy;

  auto first_wheel = wheel_observation(1.0, 0.0, 0.0);
  first_wheel.angular_velocity.z() = 0.4;
  ASSERT_TRUE(
    backend.update(
      &first_wheel, lio_observation(1.0, 0.0, 0.0),
      {}, nullptr, healthy).success);

  auto second_wheel = wheel_observation(1.1, 0.1, 0.0);
  second_wheel.angular_velocity.z() = 0.6;
  const auto result = backend.update(
    &second_wheel, lio_observation(1.1, 0.1, 0.0),
    {}, nullptr, healthy);

  ASSERT_TRUE(result.success);
  EXPECT_DOUBLE_EQ(result.angular_velocity.z(), 0.6);
}

TEST(FusionBackend, AlignsBeforeReintroducingRecoveredLio)
{
  FusionBackend backend(test_parameters());

  const auto first = lio_observation(1.0, 0.0, 0.0);
  ASSERT_TRUE(
    backend.update(nullptr, first, {}, nullptr, LioHealthResult{}).success);

  LioHealthResult failed;
  failed.state = LioHealthState::FAILED;
  failed.accept_factor = false;
  const auto failed_pose = lio_observation(1.1, 5.0, 0.0);
  ASSERT_TRUE(
    backend.update(nullptr, failed_pose, {}, nullptr, failed).success);

  LioHealthResult recovering;
  recovering.state = LioHealthState::RECOVERING;
  recovering.accept_factor = true;
  recovering.requires_alignment = true;
  const auto recovered_pose = lio_observation(1.2, 20.0, 0.0);
  const auto aligned = backend.update(
    nullptr, recovered_pose, {}, nullptr, recovering);

  ASSERT_TRUE(aligned.success);
  EXPECT_FALSE(aligned.used_lio);
  EXPECT_NEAR(aligned.pose.translation().x(), 0.0, 1.0e-3);
}

TEST(FusionBackend, SupportsWheelOnlyProfile)
{
  auto parameters = test_parameters();
  parameters.use_wheel_odom = true;
  parameters.use_fast_lio = false;

  FusionBackend backend(parameters);
  const LioHealthResult health;
  const auto first = wheel_observation(1.0, 0.0, 0.0);
  const auto second = wheel_observation(1.1, 0.4, -0.2, 0.1);

  const auto first_result = backend.update(
    &first, static_cast<const LioObservation *>(nullptr), {}, nullptr, health);
  ASSERT_TRUE(first_result.success);
  EXPECT_TRUE(first_result.initialized);

  const auto second_result = backend.update(
    &second, static_cast<const LioObservation *>(nullptr), {}, nullptr, health);
  ASSERT_TRUE(second_result.success);
  EXPECT_TRUE(second_result.used_wheel);
  EXPECT_FALSE(second_result.used_lio);
  EXPECT_NEAR(second_result.pose.translation().x(), 0.4, 1.0e-3);
  EXPECT_NEAR(second_result.pose.translation().y(), -0.2, 1.0e-3);
  EXPECT_NEAR(second_result.pose.rotation().yaw(), 0.1, 1.0e-3);
}

TEST(FusionBackend, SupportsLioOnlyProfile)
{
  auto parameters = test_parameters();
  parameters.use_wheel_odom = false;
  parameters.use_fast_lio = true;

  FusionBackend backend(parameters);
  const LioHealthResult health;
  const auto first = lio_observation(1.0, 0.0, 0.0);
  const auto second = lio_observation(1.1, 0.3, 0.1, 0.2);

  const auto first_result = backend.update(
    nullptr, first, {}, nullptr, health);
  ASSERT_TRUE(first_result.success);

  const auto second_result = backend.update(
    nullptr, second, {}, nullptr, health);
  ASSERT_TRUE(second_result.success);
  EXPECT_FALSE(second_result.used_wheel);
  EXPECT_TRUE(second_result.used_lio);
  EXPECT_NEAR(second_result.pose.translation().x(), 0.3, 1.0e-3);
  EXPECT_NEAR(second_result.pose.translation().y(), 0.1, 1.0e-3);
  EXPECT_NEAR(second_result.pose.rotation().yaw(), 0.2, 1.0e-3);
}

TEST(FusionBackend, GaussianKernelDoesNotWrapNoiseModel)
{
  auto parameters = test_parameters();
  parameters.robust_kernel = "gaussian";
  FusionBackend backend(parameters);
  const LioHealthResult health;

  ASSERT_TRUE(
    backend.update(
      nullptr, lio_observation(1.0, 0.0, 0.0), {}, nullptr, health).success);
  const auto result = backend.update(
    nullptr, lio_observation(1.1, 0.1, 0.0), {}, nullptr, health);
  ASSERT_TRUE(result.success);
  EXPECT_TRUE(result.used_lio);
}

TEST(FusionBackend, RecordsSemanticFactorsUsedByEachUpdate)
{
  auto parameters = test_parameters();
  parameters.use_planar_prior = true;

  FusionBackend backend(parameters);
  const LioHealthResult healthy;
  const auto first_wheel = wheel_observation(1.0, 0.0, 0.0);
  const auto first_lio = lio_observation(1.0, 0.0, 0.0);
  const auto first = backend.update(
    &first_wheel,
    first_lio,
    {}, nullptr, healthy);

  ASSERT_TRUE(first.success);
  ASSERT_EQ(first.factors_used.size(), 1U);
  EXPECT_TRUE(has_factor(first, FactorType::InitializationPrior));

  const auto second_wheel = wheel_observation(1.1, 0.1, 0.0);
  const auto second_lio = lio_observation(1.1, 0.1, 0.0);
  const auto second = backend.update(
    &second_wheel,
    second_lio,
    {}, nullptr, healthy);

  ASSERT_TRUE(second.success);
  EXPECT_TRUE(has_factor(second, FactorType::WheelBetween));
  EXPECT_TRUE(has_factor(second, FactorType::LioBetween));
  EXPECT_TRUE(has_factor(second, FactorType::PlanarMotion));
  EXPECT_TRUE(has_factor(second, FactorType::PlanarReference));
  EXPECT_FALSE(has_factor(second, FactorType::InitializationPrior));
}

TEST(FusionBackend, DoesNotRecordLioFactorForRecoveryAlignmentFrame)
{
  FusionBackend backend(test_parameters());
  ASSERT_TRUE(
    backend.update(
      nullptr, lio_observation(1.0, 0.0, 0.0), {}, nullptr,
      LioHealthResult{}).success);

  LioHealthResult failed;
  failed.state = LioHealthState::FAILED;
  failed.accept_factor = false;
  ASSERT_TRUE(
    backend.update(
      nullptr, lio_observation(1.1, 5.0, 0.0), {}, nullptr, failed).success);

  LioHealthResult recovering;
  recovering.state = LioHealthState::RECOVERING;
  recovering.accept_factor = true;
  recovering.requires_alignment = true;
  const auto result = backend.update(
    nullptr, lio_observation(1.2, 20.0, 0.0), {}, nullptr, recovering);

  ASSERT_TRUE(result.success);
  EXPECT_FALSE(has_factor(result, FactorType::LioBetween));
  EXPECT_FALSE(has_factor(result, FactorType::LioPrior));
  EXPECT_TRUE(has_factor(result, FactorType::FallbackPrior));
}

TEST(FactorGraphBuilder, RegistersSemanticProviders)
{
  FactorGraphBuilder builder(test_parameters());
  const auto types = builder.registered_factor_types();

  EXPECT_EQ(types.size(), 7U);
  EXPECT_EQ(types.front(), FactorType::WheelBetween);
  EXPECT_EQ(types.back(), FactorType::PlanarReference);
  EXPECT_EQ(factor_type_name(FactorType::LioBetween), "lio_between");
}

TEST(FusionBackend, ExposesExplicitInitializationAndRecoveryStates)
{
  FusionBackend backend(test_parameters());
  EXPECT_EQ(
    backend.state(), FusionBackendState::WaitingForObservability);

  const auto first = backend.update(
    nullptr, lio_observation(1.0, 0.0, 0.0), {}, nullptr, LioHealthResult{});
  ASSERT_TRUE(first.success);
  EXPECT_EQ(first.backend_state, FusionBackendState::Running);

  LioHealthResult failed;
  failed.state = LioHealthState::FAILED;
  failed.accept_factor = false;
  const auto failure = backend.update(
    nullptr, lio_observation(1.1, 1.0, 0.0), {}, nullptr, failed);
  ASSERT_TRUE(failure.success);
  EXPECT_EQ(failure.backend_state, FusionBackendState::AligningRecovery);
}

}  // namespace state_fusion_backend
