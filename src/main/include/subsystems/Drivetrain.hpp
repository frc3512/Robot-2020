// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <vector>

#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/RTNotifier.h>
#include <frc/Solenoid.h>
#include <frc/Spark.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <rev/CANSparkMax.h>
#include <wpi/mutex.h>

#include "Constants.hpp"
#include "communications/PublishNode.hpp"
#include "controllers/DrivetrainController.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

/**
 * Provides an interface for this year's drive train.
 */
class Drivetrain : public SubsystemBase, public PublishNode {
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

    /**
     * Runs the control loop.
     */
    void Iterate();

    /**
     * Sets the waypoints for a generated trajectory.
     *
     * @param waypoints list of poses
     */
    void SetWaypoints(const std::vector<frc::Pose2d>& waypoints);

    /**
     * Returns whether the drivetrain controller is at the goal waypoint.
     */
    bool AtGoal() const;

    void ProcessMessage(const CommandPacket& message) override;

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

    // Controller
    DrivetrainController m_controller{
        {0.0625, 0.125, 2.5, 0.95, 0.95}, {12.0, 12.0}, Constants::kDt};
    frc::RTNotifier m_controllerThread{Constants::kControllerPrio,
                                       &Drivetrain::Iterate, this};
    std::chrono::steady_clock::time_point m_lastTime =
        std::chrono::steady_clock::time_point::min();
    std::chrono::steady_clock::time_point m_startTime =
        std::chrono::steady_clock::time_point::min();

    bool m_manualControl = true;
    wpi::mutex m_motorControllerMutex;
};

}  // namespace frc3512
