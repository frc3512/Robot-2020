// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <functional>

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/RobotController.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <wpi/MathExtras.h>

namespace frc3512 {

Robot::Robot() {
    m_autonSelector.AddMode(
        "No-op", [] {}, [] {});
    m_autonSelector.AddMode(
        "Left Side Intake", std::bind(&Robot::AutoLeftSideIntakeInit, this),
        std::bind(&Robot::AutoLeftSideIntakePeriodic, this));
    m_autonSelector.AddMode(
        "Loading Zone Drive Forward",
        std::bind(&Robot::AutoLoadingZoneDriveForwardInit, this),
        std::bind(&Robot::AutoLoadingZoneDriveForwardPeriodic, this));
    m_autonSelector.AddMode(
        "Loading Zone Shoot Three Balls",
        std::bind(&Robot::AutoLoadingZoneShootThreeInit, this),
        std::bind(&Robot::AutoLoadingZoneShootThreePeriodic, this));
    m_autonSelector.AddMode(
        "Target Zone Shoot Three Balls",
        std::bind(&Robot::AutoTargetZoneShootThreeInit, this),
        std::bind(&Robot::AutoTargetZoneShootThreePeriodic, this));
    m_autonSelector.AddMode(
        "Target Zone Shoot Six Balls",
        std::bind(&Robot::AutoTargetZoneShootSixInit, this),
        std::bind(&Robot::AutoTargetZoneShootSixPeriodic, this));
    m_autonSelector.AddMode(
        "Right Side Drive Forward",
        std::bind(&Robot::AutoRightSideDriveForwardInit, this),
        std::bind(&Robot::AutoRightSideDriveForwardPeriodic, this));
    m_autonSelector.AddMode(
        "Right Side Intake", std::bind(&Robot::AutoRightSideIntakeInit, this),
        std::bind(&Robot::AutoRightSideIntakePeriodic, this));
    m_autonSelector.AddMode(
        "Right Side Shoot Three Balls",
        std::bind(&Robot::AutoRightSideShootThreeInit, this),
        std::bind(&Robot::AutoRightSideShootThreePeriodic, this));
    m_autonSelector.AddMode(
        "Right Side Shoot Six Balls",
        std::bind(&Robot::AutoRightSideShootSixInit, this),
        std::bind(&Robot::AutoRightSideShootSixPeriodic, this));

    frc::LiveWindow::GetInstance()->DisableAllTelemetry();

    AddPeriodic([=] { ControllerPeriodic(); }, Constants::kDt, 10_ms);
}

void Robot::Shoot() {
    if (m_state == ShootingState::kIdle) {
        m_vision.TurnLEDOn();
        m_flywheel.Shoot();
        m_state = ShootingState::kStartFlywheel;
    }
}

bool Robot::IsShooting() const { return m_state != ShootingState::kIdle; }

void Robot::SimulationInit() { SubsystemBase::RunAllSimulationInit(); }

void Robot::DisabledInit() {
    SubsystemBase::RunAllDisabledInit();

    // Reset teleop shooting state machine when disabling robot
    m_flywheel.SetGoal(0_rad_per_s);
    m_vision.TurnLEDOff();
    m_timer.Stop();
    m_state = ShootingState::kIdle;
}

void Robot::AutonomousInit() {
    SubsystemBase::RunAllAutonomousInit();
    m_autonSelector.ExecAutonomousInit();
}

void Robot::TeleopInit() { SubsystemBase::RunAllTeleopInit(); }

void Robot::TestInit() { SubsystemBase::RunAllTestInit(); }

void Robot::RobotPeriodic() {
    SubsystemBase::RunAllRobotPeriodic();

    m_batteryLogger.Log(
        units::second_t{std::chrono::steady_clock::now().time_since_epoch()},
        frc::RobotController::GetInputVoltage());
}

void Robot::SimulationPeriodic() {
    SubsystemBase::RunAllSimulationPeriodic();

    frc::sim::RoboRioSim::SetVInVoltage(
        frc::sim::BatterySim::Calculate(
            {m_drivetrain.GetCurrentDraw(), m_flywheel.GetCurrentDraw()})
            .to<double>());
}

void Robot::DisabledPeriodic() { SubsystemBase::RunAllDisabledPeriodic(); }

void Robot::AutonomousPeriodic() {
    SubsystemBase::RunAllAutonomousPeriodic();
    m_autonSelector.ExecAutonomousPeriodic();

    RunShooterSM();
}

void Robot::TeleopPeriodic() {
    SubsystemBase::RunAllTeleopPeriodic();

    static frc::Joystick appendageStick2{kAppendageStick2Port};

    if (appendageStick2.GetRawButtonPressed(1)) {
        Shoot();
    }

    RunShooterSM();
}

void Robot::TestPeriodic() { SubsystemBase::RunAllTestPeriodic(); }

void Robot::ControllerPeriodic() {
    if (frc::DriverStation::GetInstance().IsEnabled()) {
        m_drivetrain.ControllerPeriodic();
        m_turret.ControllerPeriodic();
        m_flywheel.ControllerPeriodic();
    }
}

void Robot::RunShooterSM() {
    // Shooting state machine
    switch (m_state) {
        // Wait until ball(s) are fully loaded in conveyor and trigger has been
        // pushed.
        case ShootingState::kIdle: {
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
