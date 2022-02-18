// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/controller/LTVDiffDriveController.h"

#include <cmath>

#include "frc/MathUtil.h"
#include "frc/StateSpaceUtil.h"
#include "frc/controller/LinearQuadraticRegulator.h"

using namespace frc;

/**
 * States of the drivetrain system.
 */
class State {
 public:
  /// X position in global coordinate frame.
  static constexpr int kX = 0;

  /// Y position in global coordinate frame.
  static constexpr int kY = 1;

  /// Heading in global coordinate frame.
  static constexpr int kHeading = 2;

  /// Left encoder velocity.
  static constexpr int kLeftVelocity = 3;

  /// Right encoder velocity.
  static constexpr int kRightVelocity = 4;
};

LTVDiffDriveController::LTVDiffDriveController(
    const frc::LinearSystem<2, 2, 2>& plant, units::meter_t trackwidth,
    const wpi::array<double, 5>& Qelems, const wpi::array<double, 2>& Relems,
    units::second_t dt)
    : m_trackwidth{trackwidth} {
  Eigen::Matrix<double, 5, 5> A{
      {0.0, 0.0, 0.0, 0.5, 0.5},
      {0.0, 0.0, 0.0, 0.0, 0.0},
      {0.0, 0.0, 0.0, -1.0 / m_trackwidth.value(), 1.0 / m_trackwidth.value()},
      {0.0, 0.0, 0.0, plant.A(0, 0), plant.A(0, 1)},
      {0.0, 0.0, 0.0, plant.A(1, 0), plant.A(1, 1)}};
  Eigen::Matrix<double, 5, 2> B{{0.0, 0.0},
                                {0.0, 0.0},
                                {0.0, 0.0},
                                {plant.B(0, 0), plant.B(0, 1)},
                                {plant.B(1, 0), plant.B(1, 1)}};
  Eigen::Matrix<double, 5, 5> Q = frc::MakeCostMatrix(Qelems);
  Eigen::Matrix<double, 2, 2> R = frc::MakeCostMatrix(Relems);

  // dx/dt = Ax + Bu
  // 0 = Ax + Bu
  // Ax = -Bu
  // x = -A^-1 B u
  units::meters_per_second_t maxV{-plant.A().householderQr().solve(
      plant.B() * Eigen::Vector<double, 2>{12.0, 12.0})(0)};

  Eigen::Vector<double, 5> x = Eigen::Vector<double, 5>::Zero();
  for (auto velocity = -maxV; velocity < maxV; velocity += 0.01_mps) {
    x(State::kLeftVelocity) = velocity.value();
    x(State::kRightVelocity) = velocity.value();

    // The DARE is ill-conditioned if the velocity is close to zero, so don't
    // let the system stop.
    if (units::math::abs(velocity) < 1e-4_mps) {
      m_table.Insert(velocity, Eigen::Matrix<double, 2, 5>::Zero());
    } else {
      A(State::kY, State::kHeading) = velocity.value();
      m_table.Insert(velocity,
                     frc::LinearQuadraticRegulator<5, 2>{A, B, Q, R, dt}.K());
    }
  }
}

bool LTVDiffDriveController::AtReference() const {
  return std::abs(m_error(0)) < m_tolerance(0) &&
         std::abs(m_error(1)) < m_tolerance(1) &&
         std::abs(m_error(2)) < m_tolerance(2) &&
         std::abs(m_error(3)) < m_tolerance(3) &&
         std::abs(m_error(4)) < m_tolerance(4);
}

void LTVDiffDriveController::SetTolerance(
    const Pose2d& poseTolerance,
    units::meters_per_second_t leftVelocityTolerance,
    units::meters_per_second_t rightVelocityTolerance) {
  m_tolerance = Eigen::Vector<double, 5>{
      poseTolerance.X().value(), poseTolerance.Y().value(),
      poseTolerance.Rotation().Radians().value(), leftVelocityTolerance.value(),
      rightVelocityTolerance.value()};
}

Eigen::Vector<double, 2> LTVDiffDriveController::Calculate(
    const Pose2d& currentPose, units::meters_per_second_t leftVelocity,
    units::meters_per_second_t rightVelocity, const Pose2d& poseRef,
    units::meters_per_second_t leftVelocityRef,
    units::meters_per_second_t rightVelocityRef) {
  Eigen::Vector<double, 5> x{currentPose.X().value(), currentPose.Y().value(),
                             currentPose.Rotation().Radians().value(),
                             leftVelocity.value(), rightVelocity.value()};
  Eigen::Vector<double, 5> r{poseRef.X().value(), poseRef.Y().value(),
                             poseRef.Rotation().Radians().value(),
                             leftVelocityRef.value(), rightVelocityRef.value()};

  // This implements the linear time-varying differential drive controller in
  // theorem 9.6.3 of https://tavsys.net/controls-in-frc.
  units::meters_per_second_t velocity{(leftVelocity + rightVelocity) / 2.0};
  const auto& K = m_table[velocity];

  Eigen::Matrix<double, 5, 5> inRobotFrame =
      Eigen::Matrix<double, 5, 5>::Identity();
  inRobotFrame(0, 0) = std::cos(x(State::kHeading));
  inRobotFrame(0, 1) = std::sin(x(State::kHeading));
  inRobotFrame(1, 0) = -std::sin(x(State::kHeading));
  inRobotFrame(1, 1) = std::cos(x(State::kHeading));

  m_error = r - x;
  m_error(State::kHeading) =
      frc::AngleModulus(units::radian_t{m_error(State::kHeading)}).value();
  return K * inRobotFrame * m_error;
}

Eigen::Vector<double, 2> LTVDiffDriveController::Calculate(
    const Pose2d& currentPose, units::meters_per_second_t leftVelocity,
    units::meters_per_second_t rightVelocity,
    const Trajectory::State& desiredState) {
  // v = (v_r + v_l) / 2     (1)
  // w = (v_r - v_l) / (2r)  (2)
  // k = w / v               (3)
  //
  // v_l = v - wr
  // v_l = v - (vk)r
  // v_l = v(1 - kr)
  //
  // v_r = v + wr
  // v_r = v + (vk)r
  // v_r = v(1 + kr)
  return Calculate(
      currentPose, leftVelocity, rightVelocity, desiredState.pose,
      desiredState.velocity *
          (1 - (desiredState.curvature / 1_rad * m_trackwidth / 2.0)),
      desiredState.velocity *
          (1 + (desiredState.curvature / 1_rad * m_trackwidth / 2.0)));
}
