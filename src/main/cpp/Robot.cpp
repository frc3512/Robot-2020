// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <functional>

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/RobotController.h>
#include <wpi/MathExtras.h>

namespace frc3512 {

Robot::Robot() {
    m_autonSelector.AddAutoMethod(
        "No-op", [] {}, [] {});
    m_autonSelector.AddAutoMethod(
        "Loading Zone Drive Forward",
        std::bind(&Robot::AutoLoadingZoneDriveForwardInit, this),
        std::bind(&Robot::AutoLoadingZoneDriveForwardPeriodic, this));
    m_autonSelector.AddAutoMethod(
        "Loading Zone Shoot Three Balls",
        std::bind(&Robot::AutoLoadingZoneShootThreeInit, this),
        std::bind(&Robot::AutoLoadingZoneShootThreePeriodic, this));
    m_autonSelector.AddAutoMethod(
        "Target Zone Shoot Three Balls",
        std::bind(&Robot::AutoTargetZoneShootThreeInit, this),
        std::bind(&Robot::AutoTargetZoneShootThreePeriodic, this));
    m_autonSelector.AddAutoMethod(
        "Right Side Drive Forward",
        std::bind(&Robot::AutoRightSideDriveForwardInit, this),
        std::bind(&Robot::AutoRightSideDriveForwardPeriodic, this));
    m_autonSelector.AddAutoMethod(
        "Right Side Shoot Three Balls",
        std::bind(&Robot::AutoRightSideShootThreeInit, this),
        std::bind(&Robot::AutoRightSideShootThreePeriodic, this));
}

void Robot::SimulationInit() { SubsystemBase::RunAllSimulationInit(); }

void Robot::DisabledInit() {
    SubsystemBase::RunAllDisabledInit();
    ControllerSubsystemBase::Disable();

    // Reset teleop shooting state machine when disabling robot
    m_flywheel.SetGoal(0_rad_per_s);
    m_vision.TurnLEDOff();
    m_timer.Stop();
    m_state = ShootingState::kIdle;
}

void Robot::AutonomousInit() {
    SubsystemBase::RunAllAutonomousInit();
    ControllerSubsystemBase::Enable();
    m_autonSelector.ExecAutonomousInit();
}

void Robot::TeleopInit() {
    SubsystemBase::RunAllTeleopInit();

    ControllerSubsystemBase::Enable();
}

void Robot::RobotPeriodic() {
    SubsystemBase::RunAllRobotPeriodic();

    m_batteryLogger.Log(
        units::second_t{std::chrono::steady_clock::now().time_since_epoch()},
        frc::RobotController::GetInputVoltage());

    m_flywheelEntry.SetDouble(m_flywheel.GetAngle().to<double>());
    m_flywheelGoalEntry.SetDouble(m_flywheel.GetGoal().to<double>());
    m_isFlywheelOnEntry.SetBoolean(m_flywheel.IsOn());
    m_isFlywheelReadyEntry.SetBoolean(m_flywheel.IsReady());
    m_drivetrainLeftEntry.SetDouble(
        m_drivetrain.GetLeftPosition().to<double>());
    m_drivetrainRightEntry.SetDouble(
        m_drivetrain.GetRightPosition().to<double>());
    m_drivetrainGyroEntry.SetDouble(m_drivetrain.GetAngle().to<double>());
    m_turretEntry.SetDouble(m_turret.GetAngle().to<double>());
    m_upperConveyorEntry.SetBoolean(m_intake.IsUpperSensorBlocked());
    m_lowerConveyorEntry.SetBoolean(m_intake.IsLowerSensorBlocked());
}

void Robot::SimulationPeriodic() { SubsystemBase::RunAllSimulationPeriodic(); }

void Robot::DisabledPeriodic() { SubsystemBase::RunAllDisabledPeriodic(); }

void Robot::AutonomousPeriodic() {
    SubsystemBase::RunAllAutonomousPeriodic();
    m_autonSelector.ExecAutonomousPeriodic();
}

void Robot::TeleopPeriodic() {
    SubsystemBase::RunAllTeleopPeriodic();

    static frc::Joystick appendageStick2{kAppendageStick2Port};

    // Shooting state machine
    switch (m_state) {
        // Wait until ball(s) are fully loaded in conveyor and trigger has been
        // pushed.
        case ShootingState::kIdle: {
            if (appendageStick2.GetRawButtonPressed(1)) {
                m_vision.TurnLEDOn();
                m_flywheel.Shoot();
                m_state = ShootingState::kStartFlywheel;
            }
            break;
        }
        // Allow the flywheel to spin up to the correct angular velocity.
        case ShootingState::kStartFlywheel: {
            if (m_flywheel.AtGoal()) {
                // AtGoal() returns true if either the flywheel is at the goal
                // or a timeout occurred. If a timeout occurred, the flywheel
                // may not be moving due to flywheel motor issues. Only start
                // the conveyor if the flywheel is moving to avoid jams.
                if (m_flywheel.GetAngularVelocity() > 0_rad_per_s) {
                    m_timer.Reset();
                    m_timer.Start();
                    m_state = ShootingState::kStartConveyor;
                } else {
                    m_flywheel.SetGoal(0_rad_per_s);
                    m_state = ShootingState::kIdle;
                }
            }
            break;
        }
        // Feed balls until conveyor is empty and timeout has occurred.
        case ShootingState::kStartConveyor: {
            if (m_timer.HasElapsed(3_s) && !m_intake.IsUpperSensorBlocked()) {
                m_flywheel.SetGoal(0_rad_per_s);
                m_vision.TurnLEDOff();
                m_timer.Stop();
                m_state = ShootingState::kIdle;
            }
            break;
        }
    }
}

}  // namespace frc3512

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<frc3512::Robot>(); }
#endif
