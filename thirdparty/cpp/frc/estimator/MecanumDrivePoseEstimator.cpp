/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/estimator/MecanumDrivePoseEstimator.h"

#include <limits>

#include "frc/StateSpaceUtil.h"

using namespace frc;

frc::MecanumDrivePoseEstimator::MecanumDrivePoseEstimator(
    const Rotation2d& gyroAngle, const Pose2d& initialPose,
    MecanumDriveKinematics kinematics, const Vector<3>& stateStdDevs,
    const Vector<1>& localMeasurementStdDevs,
    const Vector<3>& visionMeasurementStdDevs, units::second_t nominalDt)
    : m_observer(GetObserverSystem(), StdDevMatrixToArray<3>(stateStdDevs),
                 StdDevMatrixToArray<1>(localMeasurementStdDevs), nominalDt),
      m_kinematics(kinematics),
      m_nominalDt(nominalDt) {
  // Create continuous covariance matrix for vision measurements.
  Eigen::Matrix3d visionContR =
      MakeCovMatrix<3>(StdDevMatrixToArray<3>(visionMeasurementStdDevs));

  // Create and store discrete covariance matrix for vision measurements.
  m_visionDiscR = DiscretizeR<3>(visionContR, m_nominalDt);

  // Create vision correction mechanism.
  m_visionCorrect = [&](const auto& u, const auto& y) {
    m_observer.Correct<3>(u, y, Eigen::Matrix3d::Identity(),
                          Eigen::Matrix3d::Zero(), m_visionDiscR);
  };

  // Set initial state.
  m_observer.SetXhat(PoseToVector(initialPose));

  // Calculate offsets.
  m_gyroOffset = initialPose.Rotation() - gyroAngle;
  m_previousAngle = initialPose.Rotation();
}

void frc::MecanumDrivePoseEstimator::ResetPosition(
    const Pose2d& pose, const Rotation2d& gyroAngle) {
  // Set observer state.
  m_observer.SetXhat(PoseToVector(pose));

  // Calculate offsets.
  m_previousAngle = pose.Rotation();
  m_gyroOffset = pose.Rotation() - gyroAngle;
}

Pose2d frc::MecanumDrivePoseEstimator::GetEstimatedPosition() const {
  return Pose2d{units::meter_t(m_observer.Xhat(0)),
                units::meter_t(m_observer.Xhat(1)),
                units::radian_t(m_observer.Xhat(2))};
}

void frc::MecanumDrivePoseEstimator::AddVisionMeasurement(
    const Pose2d& visionRobotPose, units::second_t timestamp) {
  m_latencyCompensator.ApplyPastMeasurement<3>(&m_observer, m_nominalDt,
                                               PoseToVector(visionRobotPose),
                                               m_visionCorrect, timestamp);
}

Pose2d frc::MecanumDrivePoseEstimator::UpdateWithTime(
    units::second_t currentTime, const Rotation2d& gyroAngle,
    const MecanumDriveWheelSpeeds& wheelSpeeds) {
  auto dt = m_prevTime >= 0_s ? currentTime - m_prevTime : m_nominalDt;
  m_prevTime = currentTime;

  auto angle = gyroAngle + m_gyroOffset;
  auto omega = (angle - m_previousAngle).Radians() / dt;

  auto chassisSpeeds = m_kinematics.ToChassisSpeeds(wheelSpeeds);
  auto fieldRelativeVelocities =
      Translation2d(chassisSpeeds.vx * 1_s, chassisSpeeds.vy * 1_s)
          .RotateBy(angle);

  Vector<3> u = frc::MakeMatrix<3, 1>(fieldRelativeVelocities.X().to<double>(),
                                      fieldRelativeVelocities.Y().to<double>(),
                                      omega.to<double>());
  m_previousAngle = angle;

  Vector<1> localY = frc::MakeMatrix<1, 1>(angle.Radians().to<double>());
  m_latencyCompensator.AddObserverState(m_observer, u, localY, currentTime);
  m_observer.Predict(u, dt);
  m_observer.Correct(u, localY);

  return GetEstimatedPosition();
}

LinearSystem<3, 3, 1>& frc::MecanumDrivePoseEstimator::GetObserverSystem() {
  static auto uMax = std::numeric_limits<double>::max();
  static auto uMin = -uMax;
  static LinearSystem<3, 3, 1> system{Eigen::Matrix3d::Zero(),
                                      Eigen::Matrix3d::Identity(),
                                      frc::MakeMatrix<1, 3>(0.0, 0.0, 1.0),
                                      frc::MakeMatrix<1, 3>(0.0, 0.0, 0.0),
                                      frc::MakeMatrix<3, 1>(uMin, uMin, uMin),
                                      frc::MakeMatrix<3, 1>(uMax, uMax, uMax)};
  return system;
}
