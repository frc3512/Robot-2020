/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <array>

#include <Eigen/Core>
#include <units.h>

#include "frc/estimator/KalmanFilterLatencyCompensator.h"
#include "frc/estimator/UnscentedKalmanFilter.h"
#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "frc/system/LinearSystem.h"

namespace frc {

template <int N>
using Vector = Eigen::Matrix<double, N, 1>;

/**
 * This class wraps an Unscented Kalman Filter to fuse latency-compensated
 * global measurements(ex. vision) with differential drive encoder measurements.
 * It will correct for noisy global measurements and encoder drift.
 *
 * <p>This class is indented to be paired with LTVDiffDriveController as it
 * provides a 10-state estimate. This can then be trimmed into 5-state
 * using Eigen::Matrix.block<>() with the operation
 * ```10-stateEstimate.block<5, 1>(0, 0)```
 * then passed into the controller as the current state estimate.
 *
 * <p>{@link DifferentialDriveStateEstimator#update} should be called every
 * robot loop (if your robot loops are faster than the default then you should
 * change the {@link
 * DifferentialDriveStateEstimator#DifferentialDriveStateEstimator(LinearSystem,
 * Matrix, Matrix, Matrix, Matrix, DifferentialDriveKinematics, double) nominal
 * delta time}.)
 * {@link DifferentialDriveStateEstimator#applyPastGlobalMeasurement} can be
 * called as infrequently as you want.
 *
 * <p>Our state-space system is:
 *
 * <p>x = [[x, y, theta, vel_l, vel_r, dist_l, dist_r, voltError_l, voltError_r,
 * angularVelError]]^T in the field coordinate system (dist_* are wheel
 * distances.)
 *
 * <p>u = [[voltage_l, voltage_r]]^T This is typically the control input of the
 * last timestep from a LTVDiffDriveController.
 *
 * <p>y = [[x, y, theta]]^T from vision, or y = [[dist_l, dist_r, theta]] from
 * encoders and gyro.
 */
class DifferentialDriveStateEstimator {
 public:
  /**
   * Constructs a DifferentialDriveStateEstimator.
   *
   * @param plant                    A {@link LinearSystem} representing a
   * differential drivetrain.
   * @param initialState             The starting state estimate.
   * @param stateStdDevs             Standard deviations of model states.
   * Increase these numbers to trust your model less.
   * @param localMeasurementStdDevs  Standard deviations of the encoder and gyro
   * measurements. Increase these numbers to trust encoder distances and gyro
   *                                 angle less.
   * @param globalMeasurementStdDevs Standard deviations of the global(vision)
   * measurements. Increase these numbers to global(vision) measurements less.
   * @param kinematics               A {@link DifferentialDriveKinematics}
   * object representing the differential drivetrain's kinematics.
   * @param nominalDtSeconds         The time in seconds between each robot
   * loop.
   */
  DifferentialDriveStateEstimator(const LinearSystem<2, 2, 2>& plant,
                                  const Vector<10>& initialState,
                                  const Vector<10>& stateStdDevs,
                                  const Vector<3>& localMeasurementStdDevs,
                                  const Vector<3>& globalMeasurementStdDevs,
                                  const DifferentialDriveKinematics& kinematics,
                                  units::second_t nominalDt = 0.02_s);

  /**
   * Applies a global measurement with a given timestamp.
   *
   * @param robotPoseMeters  The measured robot pose.
   * @param timestampSeconds The timestamp of the global measurement in seconds.
   * Note that if you don't use your own time source by calling
   *                         {@link
   * DifferentialDriveStateEstimator#updateWithTime} then you must use a
   * timestamp with an epoch since FPGA startup (i.e. the epoch of this
   * timestamp is the same epoch as
   *                         {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp
   *                         Timer.getFPGATimestamp}.) This means that you
   * should use Timer.getFPGATimestamp as your time source in this case.
   */
  void ApplyPastGlobalMeasurement(const Pose2d& visionRobotPose,
                                  units::second_t timestamp);

  /**
   * Gets the state of the robot at the current time as estimated by the
   * Unscented Kalman Filter.
   *
   * @return The robot state estimate.
   */
  Vector<10> GetEstimatedState() const;

  /**
   * Updates the the Unscented Kalman Filter using wheel encoder information,
   * robot heading and the previous control input. The control input can be
   * obtained from a
   * {@link edu.wpi.first.wpilibc.controller.LTVDiffDriveController}.
   * Note that this should be called every loop.
   *
   * @param heading       The current heading of the robot in radians.
   * @param leftPosition  The distance traveled by the left side of the robot in
   * meters.
   * @param rightPosition The distance traveled by the right side of the robot
   * in meters.
   * @param controlInput  The control input.
   * @return The robot state estimate.
   */
  Vector<10> Update(units::radian_t heading, units::meter_t leftPosition,
                    units::meter_t rightPosition,
                    const Vector<2>& controlInput);

  /**
   * Updates the the Unscented Kalman Filter using wheel encoder information,
   * robot heading and the previous control input. The control input can be
   * obtained from a
   * {@link edu.wpi.first.wpilibj.controller.LTVDiffDriveController}.
   * Note that this should be called every loop.
   *
   * @param headingRadians     The current heading of the robot in radians.
   * @param leftPosition       The distance traveled by the left side of the
   * robot in meters.
   * @param rightPosition      The distance traveled by the right side of the
   * robot in meters.
   * @param controlInput       The control input.
   * @param currentTimeSeconds Time at which this method was called, in seconds.
   * @return The robot state estimate.
   */
  Vector<10> UpdateWithTime(units::radian_t heading,
                            units::meter_t leftPosition,
                            units::meter_t rightPosition,
                            const Vector<2>& controlInput,
                            units::second_t currentTime);

  /**
   * Resets any internal state with a given initial state.
   *
   * @param initialState Initial state for state estimate.
   */
  void Reset(const Vector<10>& initialState);

  /**
   * Resets any internal state.
   */
  void Reset();

  Vector<10> Dynamics(const Vector<10>& x, const Vector<2>& u);

  static Vector<3> LocalMeasurementModel(const Vector<10>& x,
                                         const Vector<2>& u);

  static Vector<3> GlobalMeasurementModel(const Vector<10>& x,
                                          const Vector<2>& u);

 private:
  LinearSystem<2, 2, 2> m_plant;
  units::meter_t m_rb;

  units::second_t m_nominalDt;

  UnscentedKalmanFilter<10, 2, 3> m_observer;
  KalmanFilterLatencyCompensator<10, 2, 3, UnscentedKalmanFilter<10, 2, 3>>
      m_latencyCompensator;

  std::function<void(const Vector<2>& u, const Vector<3>& y)> m_globalCorrect;

  units::second_t m_prevTime = -1_s;

  Vector<3> m_localY;
  Vector<3> m_globalY;

  template <int Dim>
  static std::array<double, Dim> StdDevMatrixToArray(
      const Vector<Dim>& stdDevs);

  class State {
   public:
    static constexpr int kX = 0;
    static constexpr int kY = 1;
    static constexpr int kHeading = 2;
    static constexpr int kLeftVelocity = 3;
    static constexpr int kRightVelocity = 4;
    static constexpr int kLeftPosition = 5;
    static constexpr int kRightPosition = 6;
    static constexpr int kLeftVoltageError = 7;
    static constexpr int kRightVoltageError = 8;
    static constexpr int kAngularVelocityError = 9;
  };

  class Input {
   public:
    static constexpr int kLeftVoltage = 0;
    static constexpr int kRightVoltage = 1;
  };

  class LocalOutput {
   public:
    static constexpr int kHeading = 0;
    static constexpr int kLeftPosition = 1;
    static constexpr int kRightPosition = 2;
  };

  class GlobalOutput {
   public:
    static constexpr int kX = 0;
    static constexpr int kY = 1;
    static constexpr int kYHeading = 2;
  };
};

}  // namespace frc
