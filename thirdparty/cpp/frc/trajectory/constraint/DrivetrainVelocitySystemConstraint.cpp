/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/trajectory/constraint/DrivetrainVelocitySystemConstraint.h"

using namespace frc;

units::meters_per_second_t DrivetrainVelocitySystemConstraint::MaxVelocity(
    const Pose2d& pose, curvature_t curvature,
    units::meters_per_second_t velocity) {
  // Calculate wheel velocity states from current velocity and curvature
  auto [vl, vr] = ToWheelVelocities(velocity, curvature, m_trackWidth);

  Eigen::Matrix<double, 2, 1> x;
  x << vl.to<double>(), vr.to<double>();

  // If either wheel velocity is greater than its maximum, normalize the wheel
  // speeds to within an achievable range while maintaining the curvature
  if (x(0, 0) > velocity.to<double>() || x(1, 0) > velocity.to<double>()) {
    x *= velocity.to<double>() / x.lpNorm<Eigen::Infinity>();
  }
  return units::meters_per_second_t{(x(0, 0) + x(1, 0)) / 2.0};
}

TrajectoryConstraint::MinMax
DrivetrainVelocitySystemConstraint::MinMaxAcceleration(
    const Pose2d& pose, curvature_t curvature,
    units::meters_per_second_t speed) {
  // Calculate wheel velocity states from current velocity and curvature
  auto [vl, vr] = ToWheelVelocities(speed, curvature, m_trackWidth);
  Eigen::Matrix<double, 2, 1> x;
  x << vl.to<double>(), vr.to<double>();

  Eigen::Matrix<double, 2, 1> xDot;
  Eigen::Matrix<double, 2, 1> u;

  // Get dx/dt for min u
  u << -m_maxVoltage.to<double>(), -m_maxVoltage.to<double>();
  xDot = m_system->A() * x + m_system->B() * u;
  auto minAccel =
      units::meters_per_second_squared_t{(xDot(0, 0) + xDot(1, 0)) / 2.0};

  // Get dx/dt for max u
  u << m_maxVoltage.to<double>(), m_maxVoltage.to<double>();
  xDot = m_system->A() * x + m_system->B() * u;
  auto maxAccel =
      units::meters_per_second_squared_t{(xDot(0, 0) + xDot(1, 0)) / 2.0};

  return {minAccel, maxAccel};
}
