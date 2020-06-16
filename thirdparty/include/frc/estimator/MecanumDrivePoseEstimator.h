/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <functional>

#include <Eigen/Core>
#include <units.h>

#include "frc/estimator/KalmanFilter.h"
#include "frc/estimator/KalmanFilterLatencyCompensator.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/MecanumDriveKinematics.h"
#include "frc2/Timer.h"

namespace frc {

template <int N>
using Vector = Eigen::Matrix<double, N, 1>;

/**
 * This class wraps a Kalman Filter to fuse latency-compensated vision
 * measurements with mecanum drive encoder velocity measurements. It will
 * correct for noisy measurements and encoder drift. It is intended to be an
 * easy but more accurate drop-in for MecanumDriveOdometry.
 *
 * Update() should be called every robot loop (if your loops are faster or
 * slower than the default, then you should change the nominal delta time using
 * the secondary constructor.
 *
 * AddVisionMeasurement() can be called as infrequently as you want; if you
 * never call it, then this class will behave mostly like regular encoder
 * odometry.
 *
 * Our state-space system is:
 *
 * x = [[x, y, theta]]^T in the field-coordinate system.
 *
 * u = [[vx, vy, omega]]^T in the field-coordinate system.
 *
 * y = [[x, y, theta]]^T in field coords from vision, or y = [[theta]]^T from
 * the gyro.
 */
class MecanumDrivePoseEstimator {
 public:
  /**
   * Constructs a MecanumDrivePoseEstimator.
   *
   * @param gyroAngle                The current gyro angle.
   * @param initialPose              The starting pose estimate.
   * @param stateStdDevs             Standard deviations of model states.
   *                                 Increase these numbers to trust your
   *                                 wheel and gyro velocities less.
   * @param localMeasurementStdDevs  Standard deviations of the gyro
   *                                 measurement. Increase this number to
   *                                 trust gyro angle measurements less.
   * @param visionMeasurementStdDevs Standard deviations of the encoder
   *                                 measurements. Increase these numbers
   *                                 to trust vision less.
   * @param nominalDt                The time in seconds between each robot
   *                                 loop.
   */
  MecanumDrivePoseEstimator(const Rotation2d& gyroAngle,
                            const Pose2d& initialPose,
                            MecanumDriveKinematics kinematics,
                            const Vector<3>& stateStdDevs,
                            const Vector<1>& localMeasurementStdDevs,
                            const Vector<3>& visionMeasurementStdDevs,
                            units::second_t nominalDt = 0.02_s);

  /**
   * Resets the position on the field.
   *
   * The gyroscope angle does not need to be reset here on the user's robot
   * code. The library automatically takes care of offsetting the gyro angle.
   *
   * @param pose      The position on the field that your robot is at.
   * @param gyroAngle The angle reported by the gyroscope.
   */
  void ResetPosition(const Pose2d& pose, const Rotation2d& gyroAngle);

  /**
   * Returns the robot pose at the current time as estimated by the Kalman
   * Filter.
   *
   * @return The estimated robot pose.
   */
  Pose2d GetEstimatedPosition() const;

  /**
   * Adds a vision measurement to the Kalman Filter. This will correct the
   * odometry pose estimate while still accounting for measurement noise.
   *
   * This method can be called as infrequently as you want, as long as you are
   * calling Update() every loop.
   *
   * @param visionRobotPose The pose of the robot as measured by the vision
   * camera.
   * @param timestamp       The timestamp of the vision measurement in seconds.
   *                        Note that if you don't use your own time source by
   *                        calling UpdateWithTime(), then you must use a
   *                        timestamp with an epoch since FPGA startup (i.e.
   *                        the epoch of this timestamp is the same epoch as
   *                        frc2::Timer::GetFPGATimestamp()).
   */
  void AddVisionMeasurement(const Pose2d& visionRobotPose,
                            units::second_t timestamp);

  /**
   * Updates the Kalman Filter using only wheel encoder information. Note that
   * this should be called every loop.
   *
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelSpeeds The speeds of each wheel of the mecanumd rive.
   *
   * @return The estimated pose.
   */
  Pose2d Update(const Rotation2d& gyroAngle,
                const MecanumDriveWheelSpeeds& wheelSpeeds) {
    return UpdateWithTime(frc2::Timer::GetFPGATimestamp(), gyroAngle,
                          wheelSpeeds);
  }

  /**
   * Updates the Kalman Filter using only wheel encoder information. Note that
   * this should be called every loop.
   *
   * @param currentTime The current time.
   * @param gyroAngle The angle reported by the gyroscope.
   * @param wheelSpeeds The speeds of each wheel of the mecanumd rive.
   *
   * @return The estimated pose.
   */
  Pose2d UpdateWithTime(units::second_t currentTime,
                        const Rotation2d& gyroAngle,
                        const MecanumDriveWheelSpeeds& wheelSpeeds);

 private:
  KalmanFilter<3, 3, 1> m_observer;
  MecanumDriveKinematics m_kinematics;
  std::function<void(const Vector<3>& u, const Vector<3>& y)> m_visionCorrect;
  KalmanFilterLatencyCompensator<3, 3, 1, KalmanFilter<3, 3, 1>>
      m_latencyCompensator;

  units::second_t m_nominalDt;
  units::second_t m_prevTime = -1_s;

  Rotation2d m_gyroOffset;
  Rotation2d m_previousAngle;

  Eigen::Matrix3d m_visionDiscR;

  static LinearSystem<3, 3, 1>& GetObserverSystem();

  template <int Dim>
  static std::array<double, Dim> StdDevMatrixToArray(
      const Vector<Dim>& vector) {
    std::array<double, Dim> array;
    for (size_t i = 0; i < Dim; ++i) {
      array[i] = vector(i);
    }
    return array;
  }
};

}  // namespace frc
