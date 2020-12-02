// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/RobotController.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>

#include "Constants.hpp"

namespace frc3512 {

Robot::Robot() {
    m_autonChooser.AddAutonomous(
        "Left Side Intake", [=] { AutoLeftSideIntakeInit(); },
        [=] { AutoLeftSideIntakePeriodic(); });
    m_autonChooser.AddAutonomous(
        "Loading Zone Drive Forward",
        [=] { AutoLoadingZoneDriveForwardInit(); },
        [=] { AutoLoadingZoneDriveForwardPeriodic(); });
    m_autonChooser.AddAutonomous(
        "Loading Zone Shoot Three Balls",
        [=] { AutoLoadingZoneShootThreeInit(); },
        [=] { AutoLoadingZoneShootThreePeriodic(); });
    m_autonChooser.AddAutonomous(
        "Target Zone Shoot Three Balls",
        [=] { AutoTargetZoneShootThreeInit(); },
        [=] { AutoTargetZoneShootThreePeriodic(); });
    m_autonChooser.AddAutonomous(
        "Target Zone Shoot Six Balls", [=] { AutoTargetZoneShootSixInit(); },
        [=] { AutoTargetZoneShootSixPeriodic(); });
    m_autonChooser.AddAutonomous(
        "Right Side Drive Forward", [=] { AutoRightSideDriveForwardInit(); },
        [=] { AutoRightSideDriveForwardPeriodic(); });
    m_autonChooser.AddAutonomous(
        "Right Side Shoot Three Balls", [=] { AutoRightSideShootThreeInit(); },
        [=] { AutoRightSideShootThreePeriodic(); });
    m_autonChooser.AddAutonomous(
        "Right Side Shoot Six Balls", [=] { AutoRightSideShootSixInit(); },
        [=] { AutoRightSideShootSixPeriodic(); });
    m_autonChooser.AddAutonomous(
        "Right Side Intake", [=] { AutoRightSideIntakeInit(); },
        [=] { AutoRightSideIntakePeriodic(); });

    frc::LiveWindow::GetInstance()->DisableAllTelemetry();

    AddPeriodic([=] { ControllerPeriodic(); }, Constants::kDt, 7.5_ms);
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
    m_autonChooser.RunAutonomousInit();
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

    frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate(
        {m_drivetrain.GetCurrentDraw(), m_flywheel.GetCurrentDraw()}));
}

void Robot::DisabledPeriodic() { SubsystemBase::RunAllDisabledPeriodic(); }

void Robot::AutonomousPeriodic() {
    SubsystemBase::RunAllAutonomousPeriodic();
    m_autonChooser.RunAutonomousPeriodic();

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
            if (m_timer.HasElapsed(2_s) && !m_intake.IsUpperSensorBlocked()) {
                m_flywheel.SetGoal(0_rad_per_s);
                m_vision.TurnLEDOff();
                m_timer.Stop();
                m_state = ShootingState::kIdle;
            }
            break;
        }
    }
}

void Robot::SelectAutonomous(wpi::StringRef name) {
    m_autonChooser.SelectAutonomous(name);
}

const std::vector<std::string>& Robot::GetAutonomousNames() const {
    return m_autonChooser.GetAutonomousNames();
}

void Robot::ExpectAutonomousEndConds() {
    if constexpr (IsSimulation()) {
        EXPECT_TRUE(m_drivetrain.AtGoal());
        EXPECT_EQ(m_flywheel.GetGoal(), 0_rad_per_s);
        EXPECT_TRUE(m_turret.AtGoal());
    }
}

}  // namespace frc3512

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<frc3512::Robot>(); }
#endif
