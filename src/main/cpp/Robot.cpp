// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <cmath>
#include <functional>

#include <frc/DriverStation.h>
#include <frc/RobotController.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <wpi/MathExtras.h>
#include <wpi/raw_ostream.h>

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

    nt::NetworkTableInstance inst = nt::NetworkTableInstance::GetDefault();
    std::shared_ptr<nt::NetworkTable> table = inst.GetTable("Diagnostics");
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

    // Consumes button presses made in disabled
    m_driveStick1.Update();
    m_driveStick2.Update();
    m_appendageStick1.Update();
    m_appendageStick2.Update();

    ControllerSubsystemBase::Enable();
}

void Robot::RobotPeriodic() {
    SubsystemBase::RunAllRobotPeriodic();
    m_batteryLogger.Log(
        units::second_t{std::chrono::steady_clock::now().time_since_epoch()},
        frc::RobotController::GetInputVoltage());
}

void Robot::DisabledPeriodic() {
    SubsystemBase::RunAllDisabledPeriodic();

    m_flywheelEntry.SetDouble(m_flywheel.GetAngle().to<double>());
    m_drivetrainLeftEntry.SetDouble(
        m_drivetrain.GetLeftPosition().to<double>());
    m_drivetrainRightEntry.SetDouble(
        m_drivetrain.GetRightPosition().to<double>());
    m_drivetrainGyroEntry.SetDouble(m_drivetrain.GetAngle().to<double>());
    m_turretEntry.SetDouble(m_turret.GetAngle().to<double>());
    m_upperConveyorEntry.SetBoolean(m_intake.IsUpperSensorBlocked());
    m_lowerConveyorEntry.SetBoolean(m_intake.IsLowerSensorBlocked());
}

void Robot::AutonomousPeriodic() {
    SubsystemBase::RunAllAutonomousPeriodic();
    m_autonSelector.ExecAutonomousPeriodic();
}

void Robot::TeleopPeriodic() {
    SubsystemBase::RunAllTeleopPeriodic();

    m_driveStick1.Update();
    m_driveStick2.Update();
    m_appendageStick1.Update();
    m_appendageStick2.Update();

    // Drivetrain
    double y = m_driveStick1.GetY();
    double x = m_driveStick2.GetX();

    if (m_driveStick1.GetRawButton(1)) {
        y *= 0.5;
        x *= 0.5;
    }
    m_drivetrain.Drive(y, x, m_driveStick2.GetRawButton(2));

    // Turret manual override
    if (m_appendageStick1.GetRawButtonPressed(11)) {
        m_turret.SetManualOverride();
    }

    // Turrret manual spin
    int pov = m_appendageStick1.GetPOV();
    if (pov == 90) {
        m_turret.SetDirection(Turret::Direction::kCW);
    } else if (pov == 270) {
        m_turret.SetDirection(Turret::Direction::kCCW);
    } else {
        m_turret.SetDirection(Turret::Direction::kNone);
    }

    // Intake
    if (m_appendageStick2.GetRawButtonPressed(4)) {
        m_intake.SetArmMotor(Intake::ArmMotorDirection::kIntake);
        m_intake.SetFunnel(0.4);
    } else if (m_appendageStick2.GetRawButtonPressed(6)) {
        m_intake.SetArmMotor(Intake::ArmMotorDirection::kOuttake);
        m_intake.SetFunnel(-0.4);
    } else if (m_appendageStick2.GetRawButtonReleased(4)) {
        m_intake.SetArmMotor(Intake::ArmMotorDirection::kIdle);
        m_intake.SetFunnel(0.0);
    } else if (m_appendageStick2.GetRawButtonReleased(6)) {
        m_intake.SetArmMotor(Intake::ArmMotorDirection::kIdle);
        m_intake.SetFunnel(0.0);
    } else if (m_appendageStick2.GetRawButtonPressed(3)) {
        if (m_intake.IsDeployed()) {
            m_intake.Stow();
        } else {
            m_intake.Deploy();
        }
    }

    // Climber traverser
    if (m_appendageStick2.GetRawButton(2)) {
        static constexpr double kMinInput = 0.5;

        // This equation rescales the following function
        //
        //   [-1 .. 0) -> [-1 .. 0) and 0 -> 0 and (0 .. 1] -> (0 .. 1]
        //
        // to
        //
        //   [-1 .. 0) -> [-1 .. -0.5) and 0 -> 0 and (0 .. 1] -> (0.5 .. 1]
        //
        // This provides a minimum input of 0.5 in either direction to overcome
        // friction while still linearly increasing to 1.
        double x = m_appendageStick2.GetX();
        m_climber.SetTransverser((1.0 - kMinInput) * x +
                                 kMinInput * wpi::sgn(x));
    } else {
        m_climber.SetTransverser(0.0);
    }

    // Climber elevator
    if (m_appendageStick1.GetRawButton(1)) {
        m_climber.SetElevator(std::abs(m_appendageStick1.GetY()));
    } else {
        m_climber.SetElevator(0.0);
    }

    // Shooting state machine
    switch (m_state) {
        // Wait until ball(s) are fully loaded in conveyor and trigger has been
        // pushed.
        case ShootingState::kIdle: {
            if (m_appendageStick2.GetRawButtonPressed(1)) {
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
