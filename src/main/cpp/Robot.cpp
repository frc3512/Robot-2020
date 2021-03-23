// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#if !defined(_MSC_VER)
#include <unistd.h>
#endif  // !defined(_MSC_VER)

#include <cstdlib>
#include <stdexcept>

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/Notifier.h>
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
    m_autonChooser.AddAutonomous("Left Side Shoot Ten Balls",
                                 [=] { AutoLeftSideShootTen(); });
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
    m_autonChooser.AddAutonomous("Right Side Shoot Eight Balls",
                                 [=] { AutoRightSideShootEight(); });
    if constexpr (Constants::Robot::kAtHomeChallenge) {
        m_autonChooser.AddAutonomous(
            "AutoNav Bounce", [=] { AutoNavBounce(); }, false);
        m_autonChooser.AddAutonomous(
            "AutoNav Barrel Racing", [=] { AutoNavBarrelRacing(); }, false);
        m_autonChooser.AddAutonomous(
            "AutoNav Slalom", [=] { AutoNavSlalom(); }, false);
        m_autonChooser.AddAutonomous(
            "Galactic Search A", [=] { AutoGalacticSearchA(); }, false);
        m_autonChooser.AddAutonomous(
            "Galactic Search B", [=] { AutoGalacticSearchB(); }, false);
    }

    frc::DriverStation::GetInstance().SilenceJoystickConnectionWarning(true);
    frc::LiveWindow::GetInstance()->DisableAllTelemetry();

    // |  Subsystem | Duration (ms) | Offset (ms) | Allocation (ms) |
    // |------------|---------------|-------------|-----------------|
    // | **Total**  | 5.0           | N/A         | 5.0             |
    // | TimedRobot | 0.6           | 0.0         | 0.8             |
    // | Drivetrain | 1.9           | 0.8         | 2.5             |
    // | Turret     | 0.6           | 3.3         | 0.8             |
    // | Flywheel   | 0.6           | 4.1         | 0.8             |
    // | **Free**   | 0.1           | 4.9         | 0.1             |
    AddPeriodic(
        [=] {
            if (IsEnabled()) {
                m_drivetrain.ControllerPeriodic();
            }
        },
        Constants::kDt, 0.8_ms);
    AddPeriodic(
        [=] {
            if (IsEnabled()) {
                m_flywheel.ControllerPeriodic();
            }
        },
        Constants::kDt, 3.3_ms);
    AddPeriodic(
        [=] {
            if (IsEnabled()) {
                m_turret.ControllerPeriodic();
            }
        },
        Constants::kDt, 4.1_ms);

    if constexpr (!IsSimulation()) {
        // crond occasionally uses 50% CPU and there's no cronjobs to run
#if !defined(_MSC_VER)
        setuid(0);
#endif  // !defined(_MSC_VER)
        int status = std::system("/etc/init.d/crond stop");
        if (status != 0) {
            throw std::runtime_error(
                fmt::format("Failed to stop crond ({})", status));
        }
    }

    if (!frc::Notifier::SetHALThreadPriority(true, 40)) {
        throw std::runtime_error(
            "Setting HAL Notifier RT priority to 40 failed\n");
    }

    if (!frc::SetCurrentThreadPriority(true, Constants::kControllerPrio)) {
        throw std::runtime_error(
            fmt::format("Setting TimedRobot RT priority to {} failed\n",
                        Constants::kControllerPrio));
    }
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

bool Robot::IsShooting() const { return m_state != ShootingState::kIdle; }

bool Robot::FlywheelAtGoal() const { return m_flywheel.AtGoal(); }

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
        if (m_autonChooser.CheckSelectedAutonomousEnds()) {
            EXPECT_FALSE(m_autonChooser.IsSuspended())
                << "Autonomous mode didn't finish within the autonomous period";
            EXPECT_TRUE(m_turret.AtGoal());
        }

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
