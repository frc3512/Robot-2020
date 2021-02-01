// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/trajectory/constraint/DifferentialDriveVelocitySystemConstraint.h"

#include <algorithm>
#include <limits>

#include <wpi/MathExtras.h>

#include "units/acceleration.h"

using namespace frc;

DifferentialDriveVelocitySystemConstraint::
    DifferentialDriveVelocitySystemConstraint(
        LinearSystem<2, 2, 2> system, DifferentialDriveKinematics kinematics,
        units::volt_t maxVoltage)
    : m_system(system), m_kinematics(kinematics), m_maxVoltage(maxVoltage) {
  Eigen::Matrix<double, 2, 1> u;
  u << m_maxVoltage.to<double>(), m_maxVoltage.to<double>();
  Eigen::Matrix<double, 2, 1> maxX =
      -m_system.A().householderQr().solve(m_system.B() * u);
  m_maxVelocity = units::meters_per_second_t{maxX(0)};
}

units::meters_per_second_t
DifferentialDriveVelocitySystemConstraint::MaxVelocity(
    const Pose2d& pose, units::curvature_t curvature,
    units::meters_per_second_t velocity) const {
  auto [vl, vr] =
      m_kinematics.ToWheelSpeeds({velocity, 0_mps, velocity * curvature});

  Eigen::Matrix<double, 2, 1> x;
  x << vl.to<double>(), vr.to<double>();

  // If either wheel velocity is greater than its maximum, normalize the wheel
  // speeds to within an achievable range while maintaining the curvature
  if (std::abs(x(0)) > m_maxVelocity.to<double>() ||
      std::abs(x(1)) > m_maxVelocity.to<double>()) {
    x *= m_maxVelocity.to<double>() / x.lpNorm<Eigen::Infinity>();
  }
  return units::meters_per_second_t{(x(0) + x(1)) / 2.0};
}

TrajectoryConstraint::MinMax
DifferentialDriveVelocitySystemConstraint::MinMaxAcceleration(
    const Pose2d& pose, units::curvature_t curvature,
    units::meters_per_second_t speed) const {
  auto wheelSpeeds =
      m_kinematics.ToWheelSpeeds({speed, 0_mps, speed * curvature});

  Eigen::Matrix<double, 2, 1> x;
  x << wheelSpeeds.left.to<double>(), wheelSpeeds.right.to<double>();

  Eigen::Matrix<double, 2, 1> u;
  Eigen::Matrix<double, 2, 1> xDot;

  // dx/dt for minimum u
  u << -m_maxVoltage.to<double>(), -m_maxVoltage.to<double>();
  xDot = m_system.A() * x + m_system.B() * u;
  units::meters_per_second_squared_t minChassisAcceleration{
      (xDot(0) + xDot(1)) / 2.0};

  // dx/dt for maximum u
  u << m_maxVoltage.to<double>(), m_maxVoltage.to<double>();
  xDot = m_system.A() * x + m_system.B() * u;
  units::meters_per_second_squared_t maxChassisAcceleration{
      (xDot(0) + xDot(1)) / 2.0};

  return {minChassisAcceleration, maxChassisAcceleration};
}
