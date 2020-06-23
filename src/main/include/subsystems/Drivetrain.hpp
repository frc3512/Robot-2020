// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <vector>

#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <rev/CANSparkMax.h>
#include <wpi/mutex.h>

#include "Constants.hpp"
#include "controllers/DrivetrainController.hpp"
#include "subsystems/ControllerSubsystemBase.hpp"

namespace frc3512 {

/**
 * Provides an interface for this year's drive train.
 */
class Drivetrain : public ControllerSubsystemBase {
public:
    Drivetrain();
    Drivetrain(const Drivetrain&) = delete;
    Drivetrain& operator=(const Drivetrain&) = delete;

    /**
     * Drives robot with given speed and turn values [-1..1].
     * This is a convenience function for use in Operator Control.
     */
    void Drive(double throttle, double turn, bool isQuickTurn = false);

    /**
     * Directly set wheel speeds (see GearBox::SetManual(double)).
     *
     * @param value speeds [0..1]
     */
    void SetLeftManual(double value);

    /**
     * Directly set wheel speeds (see GearBox::SetManual(double)).
     *
     * @param value speeds [0..1]
     */
    void SetRightManual(double value);
    /**
     * Returns gyro angle.
     *
     * @return angle in degrees
     */
    units::radian_t GetAngle() const;

    /**
     * Returns gyro angular rate.
     *
     * @return angular rate in degrees per second
     */
    units::radians_per_second_t GetAngularRate() const;

    /**
     * Resets gyro.
     */
    void ResetGyro();

    /**
     * Calibrates gyro.
     */
    void CalibrateGyro();

    /**
     * Returns left encoder displacement.
     *
     * @return left displacement
     */
    units::meter_t GetLeftPosition() const;

    /**
     * Returns right encoder displacement.
     *
     * @return right displacement
     */
    units::meter_t GetRightPosition() const;

    /**
     * Returns right encoder displacement.
     *
     * @return left rate
     */
    units::meters_per_second_t GetLeftVelocity() const;

    /**
     * Returns right encoder displacement.
     *
     * @return right rate
     */
    units::meters_per_second_t GetRightVelocity() const;

    /**
     * Resets encoders.
     */
    void ResetEncoders();

    /**
     * Resets all sensors and controller.
     */
    void Reset(const frc::Pose2d& initialPose = frc::Pose2d());

    /**
     * Enables the controller.
     */
    void EnableController();

    /**
     * Disables the controller.
     */
    void DisableController();

    /**
     * Returns true if controller is enabled and false if controller is
     * disabled.
     *
     * @return whether or not the controller is enabled
     */
    bool IsControllerEnabled() const;

    void ControllerPeriodic() override;

    /**
     * Sets the waypoints for a generated trajectory.
     *
     * @param start    Starting pose.
     * @param interior Intermediate waypoints excluding heading.
     * @param end      Ending pose.
     */
    void SetWaypoints(const frc::Pose2d& start,
                      const std::vector<frc::Translation2d>& interior,
                      const frc::Pose2d& end);

    /**
     * Sets the waypoints for a generated trajectory.
     *
     * @param start    Starting pose.
     * @param interior Intermediate waypoints excluding heading.
     * @param end      Ending pose.
     * @param config   TrajectoryConfig for this trajectory. This can include
     *                 constraints on the trajectory dynamics. If adding custom
     *                 constraints, it is recommended to start with the config
     *                 returned by MakeTrajectoryConfig() so differential drive
     *                 dynamics constraints are included automatically.
     */
    void SetWaypoints(const frc::Pose2d& start,
                      const std::vector<frc::Translation2d>& interior,
                      const frc::Pose2d& end, frc::TrajectoryConfig& config);

    /**
     * Returns whether the drivetrain controller is at the goal waypoint.
     */
    bool AtGoal() const;

    Eigen::Matrix<double, 10, 1> GetNextXhat() const;

    void DisabledInit() override { DisableController(); }

    void AutonomousInit() override {
        EnableController();
        m_controller.SetOpenLoop(false);
    }

    void TeleopInit() override { m_controller.SetOpenLoop(true); }

    void ProcessMessage(const HIDPacket& message) override;

private:
    // Left gearbox used in position PID
    rev::CANSparkMax m_leftSlave{Constants::Drivetrain::kLeftSlavePort,
                                 rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_leftMaster{Constants::Drivetrain::kLeftMasterPort,
                                  rev::CANSparkMax::MotorType::kBrushless};
    frc::SpeedControllerGroup m_leftGrbx{m_leftMaster, m_leftSlave};

    // Right gearbox used in position PID
    rev::CANSparkMax m_rightSlave{Constants::Drivetrain::kRightSlavePort,
                                  rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_rightMaster{Constants::Drivetrain::kRightMasterPort,
                                   rev::CANSparkMax::MotorType::kBrushless};
    frc::SpeedControllerGroup m_rightGrbx{m_rightMaster, m_rightSlave};

    // Left gearbox used in position PID
    frc::Encoder m_leftEncoder{Constants::Drivetrain::kLeftEncoderA,
                               Constants::Drivetrain::kLeftEncoderB};

    // Right gearbox used in position PID
    frc::Encoder m_rightEncoder{Constants::Drivetrain::kRightEncoderA,
                                Constants::Drivetrain::kRightEncoderB};

    frc::DifferentialDrive m_drive{m_leftGrbx, m_rightGrbx};

    // Gyro used for angle PID
    frc::ADXRS450_Gyro m_gyro;
    units::radian_t m_headingOffset = 0_rad;

    // Controller
    DrivetrainController m_controller;
    std::chrono::steady_clock::time_point m_lastTime =
        std::chrono::steady_clock::now();

    bool m_manualControl = true;
    wpi::mutex m_motorControllerMutex;
};

}  // namespace frc3512
