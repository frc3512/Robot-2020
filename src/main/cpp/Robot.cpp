// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/RobotController.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>

#include "Constants.hpp"
#include "Setup.hpp"
#include "logging/CSVUtil.hpp"

namespace frc3512 {

Robot::Robot() {
    StopCrond();

    // These warnings generate console prints that cause scheduling jitter
    frc::DriverStation::GetInstance().SilenceJoystickConnectionWarning(true);

    // This telemetry regularly causes loop overruns
    frc::LiveWindow::GetInstance()->DisableAllTelemetry();

    // Log NT data every 20ms instead of every 100ms for higher resolution
    // dashboard plots
    SetNetworkTablesFlushEnabled(true);

    m_autonChooser.AddAutonomous("Left Side Intake",
                                 [=] { AutoLeftSideIntake(); });
    m_autonChooser.AddAutonomous("Left Side Shoot Ten",
                                 [=] { AutoLeftSideShootTen(); });
    m_autonChooser.AddAutonomous("Loading Zone Drive Forward",
                                 [=] { AutoLoadingZoneDriveForward(); });
    m_autonChooser.AddAutonomous("Loading Zone Shoot Three",
                                 [=] { AutoLoadingZoneShootThree(); });
    m_autonChooser.AddAutonomous("Target Zone Shoot Three",
                                 [=] { AutoTargetZoneShootThree(); });
    m_autonChooser.AddAutonomous("Target Zone Shoot Six",
                                 [=] { AutoTargetZoneShootSix(); });
    m_autonChooser.AddAutonomous("Right Side Drive Forward",
                                 [=] { AutoRightSideDriveForward(); });
    m_autonChooser.AddAutonomous("Right Side Intake",
                                 [=] { AutoRightSideIntake(); });
    m_autonChooser.AddAutonomous("Right Side Shoot Three",
                                 [=] { AutoRightSideShootThree(); });
    m_autonChooser.AddAutonomous("Right Side Shoot Six",
                                 [=] { AutoRightSideShootSix(); });
    m_autonChooser.AddAutonomous("Right Side Shoot Eight",
                                 [=] { AutoRightSideShootEight(); });
    if constexpr (Constants::Robot::kAtHomeChallenge) {
        m_autonChooser.AddAutonomous(
            "AutoNav Bounce", [=] { AutoNavBounce(); }, 30_s);
        m_autonChooser.AddAutonomous(
            "AutoNav Barrel Racing", [=] { AutoNavBarrelRacing(); }, 30_s);
        m_autonChooser.AddAutonomous(
            "AutoNav Slalom", [=] { AutoNavSlalom(); }, 30_s);
        m_autonChooser.AddAutonomous(
            "Galactic Search A", [=] { AutoGalacticSearchA(); }, 30_s);
        m_autonChooser.AddAutonomous(
            "Galactic Search B", [=] { AutoGalacticSearchB(); }, 30_s);
    }

    // TIMESLICE ALLOCATION TABLE
    //
    // |  Subsystem | Duration (ms) | Allocation (ms) |
    // |------------|---------------|-----------------|
    // | **Total**  | 5.0           | 5.0             |
    // | TimedRobot | ?             | 2.0             |
    // | Drivetrain | 1.32          | 1.5             |
    // | Flywheel   | 0.6           | 0.7             |
    // | Turret     | 0.6           | 0.8             |
    // | **Free**   | 0.0           | N/A             |
    Schedule(
        [=] {
            if (IsEnabled()) {
                m_drivetrain.ControllerPeriodic();
            }
        },
        1.5_ms);
    Schedule(
        [=] {
            if (IsEnabled()) {
                m_flywheel.ControllerPeriodic();
            }
        },
        0.7_ms);
    Schedule(
        [=] {
            if (IsEnabled()) {
                m_turret.ControllerPeriodic();
            }
        },
        0.8_ms);
}

void Robot::Shoot(int ballsToShoot) {
    if (m_state == ShootingState::kIdle) {
        m_vision.TurnLEDOn();
        m_flywheel.SetGoalFromPose();
        m_state = ShootingState::kStartFlywheel;
        m_ballsToShoot = ballsToShoot;
        if (ballsToShoot != -1) {
            m_shootTimeout = 0.6_s * m_ballsToShoot;
        } else {
            m_shootTimeout = kMaxShootTimeout;
        }
        m_eventLogger.Log(
            fmt::format("Called Robot::Shoot({})", m_ballsToShoot));
    }
}

void Robot::Shoot(units::radians_per_second_t radsToShoot, int ballsToShoot) {
    if (m_state == ShootingState::kIdle) {
        m_flywheel.SetMoveAndShoot(false);
        m_flywheel.SetGoal(radsToShoot);
        m_state = ShootingState::kStartFlywheel;
        m_ballsToShoot = ballsToShoot;
        if (ballsToShoot != -1) {
            m_shootTimeout = 0.6_s * m_ballsToShoot;
        } else {
            m_shootTimeout = kMaxShootTimeout;
        }
        m_eventLogger.Log(
            fmt::format("Called Robot::Shoot({})", m_ballsToShoot));
    }
}

bool Robot::IsShooting() const { return m_state != ShootingState::kIdle; }

bool Robot::FlywheelAtGoal() const { return m_flywheel.AtGoal(); }

units::second_t Robot::SelectedAutonomousDuration() const {
    return m_autonChooser.SelectedAutonomousDuration();
}

void Robot::SimulationInit() { SubsystemBase::RunAllSimulationInit(); }

void Robot::DisabledInit() {
    m_autonChooser.ResumeAutonomous();
    SubsystemBase::RunAllDisabledInit();

    // Reset teleop shooting state machine when disabling robot
    m_flywheel.SetGoal(0_rad_per_s);
    m_vision.TurnLEDOff();
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

    static frc::Joystick appendageStick2{kAppendageStick2Port};

    if (IsOperatorControlEnabled() || IsTest()) {
        if (appendageStick2.GetRawButtonPressed(1)) {
            Shoot();
        }
        if (Constants::Robot::kAtHomeChallenge) {
            if (appendageStick2.GetRawButtonPressed(7)) {
                // Shoot from 5.91667 feet away
                Shoot(764_rad_per_s);
            }
            if (appendageStick2.GetRawButtonPressed(8)) {
                // Shoot from 10 feet away
                Shoot(436_rad_per_s);
            }
            if (appendageStick2.GetRawButtonPressed(10)) {
                // Shoot from 15 feet away
                Shoot(474_rad_per_s);
            }
            if (appendageStick2.GetRawButtonPressed(12)) {
                // Shoot from 20 feet away
                Shoot(498_rad_per_s);
            }
        }

        RunShooterSM();
    }

    auto batteryVoltage = frc::RobotController::GetInputVoltage();
    m_batteryLogger.Log(
        units::second_t{std::chrono::steady_clock::now().time_since_epoch()},
        batteryVoltage);
    m_batteryVoltageEntry.SetDouble(batteryVoltage);
    m_ballsToShootEntry.SetDouble(m_ballsToShoot);
}

void Robot::SimulationPeriodic() {
    SubsystemBase::RunAllSimulationPeriodic();

    if (intakeSim.Update(m_intake.IsConveyorRunning(), 20_ms)) {
        m_flywheel.SetSimAngularVelocity(0.96 *
                                         m_flywheel.GetAngularVelocity());
    }

    frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate(
        {m_drivetrain.GetCurrentDraw(), m_flywheel.GetCurrentDraw()}));
}

void Robot::DisabledPeriodic() { SubsystemBase::RunAllDisabledPeriodic(); }

void Robot::AutonomousPeriodic() {
    SubsystemBase::RunAllAutonomousPeriodic();
    m_autonChooser.ResumeAutonomous();

    RunShooterSM();
}

void Robot::TeleopPeriodic() { SubsystemBase::RunAllTeleopPeriodic(); }

void Robot::TestPeriodic() { SubsystemBase::RunAllTestPeriodic(); }

void Robot::RunShooterSM() {
    m_eventLogger.Log(
        fmt::format("Flywheel error = {}",
                    m_flywheel.GetGoal() - m_flywheel.GetAngularVelocity()),
        fmt::format("Flywheel AtGoal = {}", m_flywheel.AtGoal()));

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
                // may not be spinning due to flywheel motor issues. Only start
                // the conveyor if the flywheel is moving to avoid jams.
                if (m_flywheel.GetAngularVelocity() > 0_rad_per_s) {
                    m_timer.Reset();
                    m_timer.Start();
                    m_state = ShootingState::kStartConveyor;
                } else {
                    m_flywheel.SetGoal(0_rad_per_s);
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
                m_flywheel.AtGoal()) {
                --m_ballsToShoot;
                m_eventLogger.Log(fmt::format("Shot a ball; balls left = {}",
                                              m_ballsToShoot));
            }

            // If we shot the number of balls required or we were using a
            // timeout instead and the timeout expired, stop shooting
            // FIXME: Also stop flywheel if m_ballsToShoot == 0 when
            // Flywheel::AtGoal() is fixed
            if (m_timer.HasElapsed(m_shootTimeout) &&
                !m_intake.IsUpperSensorBlocked()) {
                m_flywheel.SetGoal(0_rad_per_s);
                m_vision.TurnLEDOff();
                m_timer.Stop();
                m_state = ShootingState::kIdle;
            }
            break;
        }
    }

    m_prevFlywheelAtGoal = m_flywheel.AtGoal();
}

void Robot::SelectAutonomous(wpi::StringRef name) {
    m_autonChooser.SelectAutonomous(name);
}

const std::vector<std::string>& Robot::GetAutonomousNames() const {
    return m_autonChooser.GetAutonomousNames();
}

void Robot::ExpectAutonomousEndConds() {
    if constexpr (IsSimulation()) {
        EXPECT_FALSE(m_autonChooser.IsSuspended())
            << "Autonomous mode didn't finish within the autonomous period";
        EXPECT_TRUE(m_turret.AtGoal());

        EXPECT_TRUE(m_drivetrain.AtGoal());

        // Verify left/right wheel velocities are zero
        EXPECT_NEAR(m_drivetrain.GetStates()(3), 0.0, 0.01);
        EXPECT_NEAR(m_drivetrain.GetStates()(4), 0.0, 0.01);

        EXPECT_EQ(m_flywheel.GetGoal(), 0_rad_per_s);
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
