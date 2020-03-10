// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <functional>

#include <frc/DriverStation.h>
#include <wpi/raw_ostream.h>

namespace frc3512 {

Robot::Robot() : PublishNode("Robot") {
    m_drivetrain.Subscribe(*this);
    m_flywheel.Subscribe(*this);
    m_turret.Subscribe(*this);
    m_intake.Subscribe(*this);
    m_vision.Subscribe(*this);
    m_climber.Subscribe(*this);

    m_flywheel.Subscribe(m_turret);

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
        "Right Side Shoot Three Balls",
        std::bind(&Robot::AutoRightSideShootThreeInit, this),
        std::bind(&Robot::AutoRightSideShootThreePeriodic, this));
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
    for (int i = 1; i <= 12; i++) {
        if (m_driveStick1.GetRawButtonPressed(i)) {
            ButtonPacket message{"DriveStick1", i, true};
            Publish(message);
        }
        if (m_driveStick2.GetRawButtonPressed(i)) {
            ButtonPacket message{"DriveStick2", i, true};
            Publish(message);
        }
        if (m_appendageStick.GetRawButtonPressed(i)) {
            ButtonPacket message{"AppendageStick", i, true};
            Publish(message);
        }
        if (m_appendageStick2.GetRawButtonPressed(i)) {
            ButtonPacket message{"AppendageStick2", i, true};
            Publish(message);
        }
    }

    ControllerSubsystemBase::Enable();
}

void Robot::TestInit() {
    wpi::outs() << "Turret Encoder reset\n";
    m_turret.ResetEncoder();
}

void Robot::RobotPeriodic() { SubsystemBase::RunAllRobotPeriodic(); }

void Robot::DisabledPeriodic() {
    SubsystemBase::RunAllDisabledPeriodic();

    wpi::outs() << "Flywheel: " << m_flywheel.GetAngle().to<double>() << "\n";
    wpi::outs() << "Drivetrain Left: "
                << m_drivetrain.GetLeftPosition().to<double>() << "\n";
    wpi::outs() << "Drivetrain Right: "
                << m_drivetrain.GetRightPosition().to<double>() << "\n";
    wpi::outs() << "Drivetrain Gyro: " << m_drivetrain.GetAngle().to<double>()
                << "\n";
    wpi::outs() << "Turret: " << m_turret.GetAngle().to<double>() << "\n";
    wpi::outs() << "Upper Conveyor: " << m_intake.IsUpperSensorBlocked()
                << "\n";
    wpi::outs() << "Lower Conveyor: " << m_intake.IsLowerSensorBlocked()
                << "\n";
}

void Robot::AutonomousPeriodic() {
    SubsystemBase::RunAllAutonomousPeriodic();
    m_autonSelector.ExecAutonomousPeriodic();
}

void Robot::TeleopPeriodic() {
    SubsystemBase::RunAllTeleopPeriodic();

    for (int i = 2; i <= 12; i++) {
        if (m_driveStick1.GetRawButtonPressed(i)) {
            ButtonPacket message{"DriveStick1", i, true};
            Publish(message);
        }
        if (m_appendageStick.GetRawButtonPressed(i)) {
            ButtonPacket message{"AppendageStick", i, true};
            Publish(message);
        }
        if (m_appendageStick2.GetRawButtonPressed(i)) {
            ButtonPacket message{"AppendageStick2", i, true};
            Publish(message);
        }
        if (m_appendageStick2.GetRawButtonReleased(i)) {
            ButtonPacket message{"AppendageStick2", i, false};
            Publish(message);
        }
    }

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

    POVPacket povMessage{"AppendageStick2", m_appendageStick.GetPOV()};
    Publish(povMessage);

    auto& ds = frc::DriverStation::GetInstance();
    HIDPacket hidMessage{"",
                         m_driveStick1.GetX(),
                         m_driveStick1.GetY(),
                         ds.GetStickButtons(0),
                         m_driveStick2.GetX(),
                         m_driveStick2.GetY(),
                         ds.GetStickButtons(1),
                         m_appendageStick.GetX(),
                         m_appendageStick.GetY(),
                         ds.GetStickButtons(2),
                         m_appendageStick2.GetX(),
                         m_appendageStick2.GetY(),
                         ds.GetStickButtons(3)};
    Publish(hidMessage);
}

}  // namespace frc3512

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<frc3512::Robot>(); }
#endif
