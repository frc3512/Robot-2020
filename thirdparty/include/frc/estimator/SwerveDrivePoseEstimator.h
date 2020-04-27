/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <array>
#include <iostream>
#include <limits>

#include <Eigen/Core>
#include <units.h>

#include "frc/StateSpaceUtil.h"
#include "frc/estimator/KalmanFilter.h"
#include "frc/estimator/KalmanFilterLatencyCompensator.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc2/Timer.h"

namespace frc {

template <int N>
using Vector = Eigen::Matrix<double, N, 1>;

/**
 * This class wraps a Kalman Filter to fuse latency-compensated vision
 * measurements with swerve drive encoder velocity and angle measurements. It
 * will correct for noisy measurements and encoder drift. It is intended to be
 * an easy drop-in replacement for SwerveDriveOdometry.
 *
 * Update() should be called every robot loop. If your loop times are faster or
 * slower than the default TimedRobot loop time, you should change it by passing
 * in an argument for nominalDt in the constructor for this class.
 *
 * AddVisionMeasurement() can be called as infrequently as you want. If you
 * never call it, this class will behave mostly like regular encoder odometry.
 *
 * Our state-space system is as follows:
 *
 * x = [[x, y, theta]]^T in the field coordinate system.
 *
 * u = [[vx, vy, omega]]^T in the field coordinate system.
 *
 * y = [[x, y, theta]]^T in the field coords from vision or y = [[theta]]^T from
 * a gyro.
 */
template <unsigned int NumModules>
class SwerveDrivePoseEstimator {
 public:
  SwerveDrivePoseEstimator(const Rotation2d& gyroAngle,
                           const Pose2d& initialPose,
                           SwerveDriveKinematics<NumModules>& kinematics,
                           const Vector<3>& stateStdDevs,
                           const Vector<1>& localMeasurementStdDevs,
                           const Vector<3>& visionMeasurementStdDevs,
                           units::second_t nominalDt = 0.02_s)
      : m_observer(GetObserverSystem(), StdDevMatrixToArray<3>(stateStdDevs),
                   StdDevMatrixToArray<1>(localMeasurementStdDevs), nominalDt),
        m_kinematics(kinematics),
        m_nominalDt(nominalDt) {
    // Construct R (covariances) matrix for vision measurements.
    Eigen::Matrix3d visionContR =
        frc::MakeCovMatrix<3>(StdDevMatrixToArray<3>(visionMeasurementStdDevs));
    m_visionDiscR = frc::DiscretizeR<3>(visionContR, m_nominalDt);

    m_visionCorrect = [&](const Vector<3>& u, const Vector<3>& y) {
      m_observer.Correct<3>(u, y, Eigen::Matrix3d::Identity(),
                            Eigen::Matrix3d::Zero(), m_visionDiscR);
    };

    m_observer.SetXhat(PoseToVector(initialPose));
    m_gyroOffset = initialPose.Rotation() - gyroAngle;
    m_previousAngle = initialPose.Rotation();
  }

  void ResetPosition(const Pose2d& pose, const Rotation2d& gyroAngle) {
    m_previousAngle = pose.Rotation();
    m_observer.SetXhat(PoseToVector(pose));
    m_gyroOffset = GetEstimatedPosition().Rotation() - gyroAngle;
  }

  Pose2d GetEstimatedPosition() const {
    return Pose2d(m_observer.Xhat(0) * 1_m, m_observer.Xhat(1) * 1_m,
                  Rotation2d(m_observer.Xhat(2) * 1_rad));
  }

  void AddVisionMeasurement(const Pose2d& visionRobotPose,
                            units::second_t timestamp) {
    m_latencyCompensator.ApplyPastMeasurement<3>(&m_observer, m_nominalDt,
                                                 PoseToVector(visionRobotPose),
                                                 m_visionCorrect, timestamp);
  }

  template <typename... ModuleState>
  Pose2d Update(const Rotation2d& gyroAngle, ModuleState&&... moduleStates) {
    return UpdateWithTime(frc2::Timer::GetFPGATimestamp(), gyroAngle,
                          moduleStates...);
  }

  template <typename... ModuleState>
  Pose2d UpdateWithTime(units::second_t currentTime,
                        const Rotation2d& gyroAngle,
                        ModuleState&&... moduleStates) {
    auto dt = m_prevTime >= 0_s ? currentTime - m_prevTime : m_nominalDt;
    m_prevTime = currentTime;

    auto angle = gyroAngle + m_gyroOffset;
    auto omega = (angle - m_previousAngle).Radians() / dt;

    auto chassisSpeeds = m_kinematics.ToChassisSpeeds(moduleStates...);
    auto fieldRelativeSpeeds =
        Translation2d(chassisSpeeds.vx * 1_s, chassisSpeeds.vy * 1_s)
            .RotateBy(angle);

    auto u =
        frc::MakeMatrix<3, 1>(fieldRelativeSpeeds.X().template to<double>(),
                              fieldRelativeSpeeds.Y().template to<double>(),
                              omega.template to<double>());
    m_previousAngle = angle;

    auto localY = frc::MakeMatrix<1, 1>(angle.Radians().template to<double>());
    m_latencyCompensator.AddObserverState(m_observer, u, localY, currentTime);

    m_observer.Predict(u, dt);
    m_observer.Correct(u, localY);

    return GetEstimatedPosition();
  }

 private:
  KalmanFilter<3, 3, 1> m_observer;
  SwerveDriveKinematics<NumModules>& m_kinematics;
  KalmanFilterLatencyCompensator<3, 3, 1, KalmanFilter<3, 3, 1>>
      m_latencyCompensator;
  std::function<void(const Vector<3>& u, const Vector<3>& y)> m_visionCorrect;

  Eigen::Matrix3d m_visionDiscR;

  units::second_t m_nominalDt;
  units::second_t m_prevTime = -1_s;

  Rotation2d m_gyroOffset;
  Rotation2d m_previousAngle;

  static LinearSystem<3, 3, 1>& GetObserverSystem() {
    static auto uMax = std::numeric_limits<double>::max();
    static auto uMin = -uMax;
    static LinearSystem<3, 3, 1> system{
        Eigen::Matrix3d::Zero(),
        Eigen::Matrix3d::Identity(),
        frc::MakeMatrix<1, 3>(0.0, 0.0, 1.0),
        frc::MakeMatrix<1, 3>(0.0, 0.0, 0.0),
        frc::MakeMatrix<3, 1>(uMin, uMin, uMin),
        frc::MakeMatrix<3, 1>(uMax, uMax, uMax)};
    return system;
  }

  template <int Dim>
  static std::array<double, Dim> StdDevMatrixToArray(
      const Vector<Dim>& vector) {
    std::array<double, Dim> array;
    for (unsigned int i = 0; i < Dim; ++i) {
      array[i] = vector(i);
    }
    return array;
  }
};

}  // namespace frc
