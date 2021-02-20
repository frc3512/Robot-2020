// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <stdexcept>

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/RobotController.h>
#include <frc/Threads.h>
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
    if constexpr (Constants::Robot::kAtHomeChallenge) {
        m_autonChooser.AddAutonomous(
            "AutoNav Bounce", [=] { AutoNavBounce(); }, false);
        m_autonChooser.AddAutonomous(
            "AutoNav Barrel Racing", [=] { AutoNavBarrelRacing(); }, false);
        m_autonChooser.AddAutonomous(
            "AutoNav Slalom", [=] { AutoNavSlalom(); }, false);
    }

    frc::DriverStation::GetInstance().SilenceJoystickConnectionWarning(true);
    frc::LiveWindow::GetInstance()->DisableAllTelemetry();

    AddPeriodic([=] { ControllerPeriodic(); }, Constants::kDt, 7.5_ms);

    if constexpr (!IsSimulation()) {
        if (!frc::SetCurrentThreadPriority(true, Constants::kControllerPrio)) {
            throw std::runtime_error(
                fmt::format("Setting RT priority to {} failed\n",
                            Constants::kControllerPrio));
        }
    }
}

void Robot::Shoot(int ballsToShoot) {
    if (m_state == ShootingState::kIdle) {
        vision.TurnLEDOn();
        flywheel.SetGoalFromPose();
        m_state = ShootingState::kStartFlywheel;
        m_ballsToShoot = ballsToShoot;
    }
}

bool Robot::IsShooting() const { return m_state != ShootingState::kIdle; }

void Robot::SimulationInit() { SubsystemBase::RunAllSimulationInit(); }

void Robot::DisabledInit() {
    m_autonChooser.ResumeAutonomous();
    SubsystemBase::RunAllDisabledInit();

    // Reset teleop shooting state machine when disabling robot
    flywheel.SetGoal(0_rad_per_s);
    vision.TurnLEDOff();
    m_timer.Stop();
    m_state = ShootingState::kIdle;
}

void Robot::AutonomousInit() {
    SubsystemBase::RunAllAutonomousInit();
    m_autonChooser.AwaitAutonomous();
}

void Robot::TeleopInit() {
    m_autonChooser.CancelAutonomous();
    SubsystemBase::RunAllTeleopInit();
}

void Robot::TestInit() {
    m_autonChooser.ResumeAutonomous();
    SubsystemBase::RunAllTestInit();
}

void Robot::RobotPeriodic() {
    SubsystemBase::RunAllRobotPeriodic();

    auto batteryVoltage = frc::RobotController::GetInputVoltage();
    m_batteryLogger.Log(
        units::second_t{std::chrono::steady_clock::now().time_since_epoch()},
        batteryVoltage);
    m_batteryVoltageEntry.SetDouble(batteryVoltage);
    m_ballsToShootEntry.SetDouble(m_ballsToShoot);
}

void Robot::SimulationPeriodic() {
    SubsystemBase::RunAllSimulationPeriodic();

    if (intakeSim.Update(intake.IsConveyorRunning(), 20_ms)) {
        flywheel.SetSimAngularVelocity(0.98 * flywheel.GetAngularVelocity());
    }

    frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate(
        {drivetrain.GetCurrentDraw(), flywheel.GetCurrentDraw()}));
}

void Robot::DisabledPeriodic() { SubsystemBase::RunAllDisabledPeriodic(); }

void Robot::AutonomousPeriodic() {
    SubsystemBase::RunAllAutonomousPeriodic();
    m_autonChooser.ResumeAutonomous();

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
    if (!frc::DriverStation::GetInstance().IsEnabled()) {
        return;
    }

    drivetrain.ControllerPeriodic();
    turret.ControllerPeriodic();
    flywheel.ControllerPeriodic();
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
            // If shooting a specific number of balls and the flywheel has gone
            // from not at the goal to at the goal, we shot a ball and have
            // recovered
            if (m_ballsToShoot > 0 && !m_prevFlywheelAtGoal &&
                flywheel.AtGoal()) {
                --m_ballsToShoot;
            }

            // If we shot the number of balls required or we were using a
            // timeout instead and the timeout expired, stop shooting
            if (m_ballsToShoot == 0 || (m_timer.HasElapsed(kShootTimeout) &&
                                        !intake.IsUpperSensorBlocked())) {
                flywheel.SetGoal(0_rad_per_s);
                vision.TurnLEDOff();
                m_timer.Stop();
                m_state = ShootingState::kIdle;
            }
            break;
        }
    }

    m_prevFlywheelAtGoal = flywheel.AtGoal();
}

void Robot::SelectAutonomous(wpi::StringRef name) {
    m_autonChooser.SelectAutonomous(name);
}

const std::vector<std::string>& Robot::GetAutonomousNames() const {
    return m_autonChooser.GetAutonomousNames();
}

void Robot::ExpectAutonomousEndConds() {
    if constexpr (IsSimulation()) {
        if (m_autonChooser.CheckSelectedAutonomousEnds()) {
            EXPECT_FALSE(m_autonChooser.IsSuspended())
                << "Autonomous mode didn't finish within the autonomous period";
        }

        EXPECT_TRUE(drivetrain.AtGoal());

        // Verify left/right wheel velocities are zero
        EXPECT_NEAR(drivetrain.GetStates()(3), 0.0, 0.01);
        EXPECT_NEAR(drivetrain.GetStates()(4), 0.0, 0.01);

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
