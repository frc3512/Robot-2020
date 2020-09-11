// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <functional>

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/RobotController.h>
#include <networktables/NetworkTableInstance.h>
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

    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("Diagnostics");
    m_flywheelEntry = table->GetEntry("Flywheel encoder");
    m_drivetrainLeftEntry = table->GetEntry("Left drivetrain encoder");
    m_drivetrainRightEntry = table->GetEntry("Right drivetrain encoder");
    m_drivetrainGyroEntry = table->GetEntry("Drivetrain angle");
    m_turretEntry = table->GetEntry("Turret angle");
    m_upperConveyorEntry = table->GetEntry("Upper conveyor sensor");
    m_lowerConveyorEntry = table->GetEntry("Lower conveyor sensor");
}

void Robot::DisabledInit() {
    SubsystemBase::RunAllDisabledInit();
    ControllerSubsystemBase::Disable();
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
    m_drivetrainLeftEntry.SetDouble(
        m_drivetrain.GetLeftPosition().to<double>());
    m_drivetrainRightEntry.SetDouble(
        m_drivetrain.GetRightPosition().to<double>());
    m_drivetrainGyroEntry.SetDouble(m_drivetrain.GetAngle().to<double>());
    m_turretEntry.SetDouble(m_turret.GetAngle().to<double>());
}

void Robot::DisabledPeriodic() {
    SubsystemBase::RunAllDisabledPeriodic();

    m_upperConveyorEntry.SetBoolean(m_intake.IsUpperSensorBlocked());
    m_lowerConveyorEntry.SetBoolean(m_intake.IsLowerSensorBlocked());
}

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
                m_timer.Reset();
                m_timer.Start();
                m_state = ShootingState::kStartConveyor;
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
