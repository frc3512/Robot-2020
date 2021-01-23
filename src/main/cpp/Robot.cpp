// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/RobotController.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>

#include "Constants.hpp"
#include "logging/CSVUtil.hpp"

namespace frc3512 {

Robot::Robot() {
    m_autonChooser.AddAutonomous("Left Side Intake",
                                 [=] { AutoLeftSideIntake(); });
    m_autonChooser.AddAutonomous("Loading Zone Drive Forward",
                                 [=] { AutoLoadingZoneDriveForward(); });
    m_autonChooser.AddAutonomous("Loading Zone Shoot Three Balls",
                                 [=] { AutoLoadingZoneShootThree(); });
    m_autonChooser.AddAutonomous("Target Zone Shoot Three Balls",
                                 [=] { AutoTargetZoneShootThree(); });
    m_autonChooser.AddAutonomous("Target Zone Shoot Six Balls",
                                 [=] { AutoTargetZoneShootSix(); });
    m_autonChooser.AddAutonomous("Right Side Drive Forward",
                                 [=] { AutoRightSideDriveForward(); });
    m_autonChooser.AddAutonomous("Right Side Intake",
                                 [=] { AutoRightSideIntake(); });
    m_autonChooser.AddAutonomous("Right Side Shoot Three Balls",
                                 [=] { AutoRightSideShootThree(); });
    m_autonChooser.AddAutonomous("Right Side Shoot Six Balls",
                                 [=] { AutoRightSideShootSix(); });

    frc::DriverStation::GetInstance().SilenceJoystickConnectionWarning(true);
    frc::LiveWindow::GetInstance()->DisableAllTelemetry();

    AddPeriodic([=] { ControllerPeriodic(); }, Constants::kDt, 7.5_ms);
}

void Robot::Shoot() {
    if (m_state == ShootingState::kIdle) {
        vision.TurnLEDOn();
        flywheel.SetGoalFromPose();
        m_state = ShootingState::kStartFlywheel;
    }
}

bool Robot::IsShooting() const { return m_state != ShootingState::kIdle; }

void Robot::SimulationInit() { SubsystemBase::RunAllSimulationInit(); }

void Robot::DisabledInit() {
    m_autonChooser.EndAutonomous();
    SubsystemBase::RunAllDisabledInit();

    // Reset teleop shooting state machine when disabling robot
    flywheel.SetGoal(0_rad_per_s);
    vision.TurnLEDOff();
    m_timer.Stop();
    m_state = ShootingState::kIdle;
}

void Robot::AutonomousInit() {
    SubsystemBase::RunAllAutonomousInit();
    m_autonChooser.AwaitStartAutonomous();
}

void Robot::TeleopInit() {
    m_autonChooser.EndAutonomous();
    SubsystemBase::RunAllTeleopInit();
}

void Robot::TestInit() {
    m_autonChooser.EndAutonomous();
    SubsystemBase::RunAllTestInit();
}

void Robot::RobotPeriodic() {
    SubsystemBase::RunAllRobotPeriodic();

    m_batteryLogger.Log(
        units::second_t{std::chrono::steady_clock::now().time_since_epoch()},
        frc::RobotController::GetInputVoltage());
}

void Robot::SimulationPeriodic() {
    SubsystemBase::RunAllSimulationPeriodic();

    frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate(
        {drivetrain.GetCurrentDraw(), flywheel.GetCurrentDraw()}));
}

void Robot::DisabledPeriodic() { SubsystemBase::RunAllDisabledPeriodic(); }

void Robot::AutonomousPeriodic() {
    SubsystemBase::RunAllAutonomousPeriodic();
    m_autonChooser.AwaitRunAutonomous();

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
        drivetrain.ControllerPeriodic();
        turret.ControllerPeriodic();
        flywheel.ControllerPeriodic();
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
            if (flywheel.AtGoal()) {
                // AtGoal() returns true if either the flywheel is at the goal
                // or a timeout occurred. If a timeout occurred, the flywheel
                // may not be spinning due to flywheel motor issues. Only start
                // the conveyor if the flywheel is moving to avoid jams.
                if (flywheel.GetAngularVelocity() > 0_rad_per_s) {
                    m_timer.Reset();
                    m_timer.Start();
                    m_state = ShootingState::kStartConveyor;
                } else {
                    flywheel.SetGoal(0_rad_per_s);
                    m_state = ShootingState::kIdle;
                    frc::DriverStation::GetInstance().ReportError(
                        "Flywheel didn't start spinning. Either the motors "
                        "aren't responding or the encoder isn't plugged in.");
                }
            }
            break;
        }
        // Feed balls until conveyor is empty and timeout has occurred.
        case ShootingState::kStartConveyor: {
            if (m_timer.HasElapsed(kShootTimeout) &&
                !intake.IsUpperSensorBlocked()) {
                flywheel.SetGoal(0_rad_per_s);
                vision.TurnLEDOff();
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
        EXPECT_TRUE(drivetrain.AtGoal());

        // Verify left/right wheel velocities are zero
        EXPECT_NEAR(drivetrain.GetStates()(3), 0.0, 1e-6);
        EXPECT_NEAR(drivetrain.GetStates()(4), 0.0, 1e-6);

        EXPECT_EQ(flywheel.GetGoal(), 0_rad_per_s);
        EXPECT_TRUE(turret.AtGoal());
    }
}

}  // namespace frc3512

#ifndef RUNNING_FRC_TESTS
int main() {
    if constexpr (frc::RobotBase::IsSimulation()) {
        frc3512::DeleteCSVs();
    }
    return frc::StartRobot<frc3512::Robot>();
}
#endif
