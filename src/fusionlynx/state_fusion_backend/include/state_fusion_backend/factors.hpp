#pragma once

/**
 * @file factors.hpp
 * @brief Custom factors used by the FusionLynx fixed-lag backend.
 * @author WenSheng Xu
 *
 * Modification history:
 * - 2026-08-21, WenSheng Xu:
 *   Added yaw-only magnetic and soft planar-prior factors. These factors keep
 *   physical sensor responsibilities explicit and avoid using MAG as a Z-axis
 *   measurement.
 */

#include <cmath>
#include <functional>

#include <boost/make_shared.hpp>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace state_fusion_backend
{

inline double wrap_angle(double angle)
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

class MagneticHeadingFactor final : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
public:
  MagneticHeadingFactor(
    gtsam::Key key,
    double heading,
    const gtsam::SharedNoiseModel & model)
  : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, key),
    heading_(heading)
  {
  }

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose,
    boost::optional<gtsam::Matrix &> H = boost::none) const override
  {
    const std::function<gtsam::Vector1(const gtsam::Pose3 &)> error =
      [this](const gtsam::Pose3 & value) {
        gtsam::Vector1 result;
        result << wrap_angle(value.rotation().yaw() - heading_);
        return result;
      };

    if (H) {
      *H = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(error, pose);
    }
    return error(pose);
  }

private:
  double heading_{0.0};
};

class PlanarPriorFactor final : public gtsam::NoiseModelFactor1<gtsam::Pose3>
{
public:
  PlanarPriorFactor(
    gtsam::Key key,
    double reference_z,
    double reference_roll,
    double reference_pitch,
    const gtsam::SharedNoiseModel & model)
  : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, key)
  , reference_z_(reference_z)
  , reference_roll_(reference_roll)
  , reference_pitch_(reference_pitch)
  {
  }

  gtsam::Vector evaluateError(
    const gtsam::Pose3 & pose,
    boost::optional<gtsam::Matrix &> H = boost::none) const override
  {
    const std::function<gtsam::Vector3(const gtsam::Pose3 &)> error =
      [this](const gtsam::Pose3 & value) {
        gtsam::Vector3 result;
        result << value.translation().z() - reference_z_,
          wrap_angle(value.rotation().roll() - reference_roll_),
          wrap_angle(value.rotation().pitch() - reference_pitch_);
        return result;
      };

    if (H) {
      *H = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Pose3>(error, pose);
    }
    return error(pose);
  }

private:
  double reference_z_{0.0};
  double reference_roll_{0.0};
  double reference_pitch_{0.0};
};

}  // namespace state_fusion_backend
