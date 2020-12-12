/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/trajectory/constraint/DifferentialDriveVelocitySystemConstraint.h"

#include <algorithm>
#include <limits>

#include <units/acceleration.h>
#include <wpi/MathExtras.h>

using namespace frc;

DifferentialDriveVelocitySystemConstraint::
    DifferentialDriveVelocitySystemConstraint(
        LinearSystem<2, 2, 2> system, DifferentialDriveKinematics kinematics,
        units::volt_t maxVoltage)
    : m_system(system), m_kinematics(kinematics), m_maxVoltage(maxVoltage) {}

units::meters_per_second_t
DifferentialDriveVelocitySystemConstraint::MaxVelocity(
    const Pose2d& pose, units::curvature_t curvature,
    units::meters_per_second_t velocity) const {
  auto [vl, vr] =
      m_kinematics.ToWheelSpeeds({velocity, 0_mps, velocity * curvature});

  Eigen::Matrix<double, 2, 1> x;
  x << vl.to<double>(), vr.to<double>();

  Eigen::Matrix<double, 2, 1> u;
  u << m_maxVoltage.to<double>(), m_maxVoltage.to<double>();
  Eigen::Matrix<double, 2, 1> maxX =
      -m_system.A().householderQr().solve(m_system.B() * u);

  // If either wheel velocity is greater than its maximum, normalize the wheel
  // speeds to within an achievable range while maintaining the curvature
  if (std::abs(x(0)) > maxX(0) || std::abs(x(1)) > maxX(1)) {
    x *= maxX(0) / x.lpNorm<Eigen::Infinity>();
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
