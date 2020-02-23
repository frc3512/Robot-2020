// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

#include <frc/DriverStation.h>
#include <wpi/raw_ostream.h>

namespace frc3512 {

Robot::Robot() : PublishNode("Robot") {
    m_drivetrain.Subscribe(*this);
    m_flywheel.Subscribe(*this);
    m_flywheel.Subscribe(m_turret);
    m_turret.Subscribe(*this);
    m_intake.Subscribe(*this);
    m_vision.Subscribe(*this);
    m_climber.Subscribe(*this);
}

void Robot::DisabledInit() {
    CommandPacket message{"DisabledInit", false};
    Publish(message);
}

void Robot::AutonomousInit() {
    CommandPacket message{"AutonomousInit", false};
    Publish(message);
}

void Robot::TeleopInit() {
    CommandPacket message{"TeleopInit", false};
    Publish(message);
    for (int i = 1; i <= 12; i++) {
        if (m_driveStick1.GetRawButtonPressed(i)) {
            ButtonPacket message{"DriveStick1", i, true};
            Publish(message);
        }
    }
}

void Robot::TestInit() {}

void Robot::RobotPeriodic() {}

void Robot::DisabledPeriodic() {
    wpi::outs() << "Flywheel: " << m_flywheel.GetAngle().to<double>() << "\n";
    wpi::outs() << "Drivetrain Left: "
                << m_drivetrain.GetLeftPosition().to<double>() << "\n";
    wpi::outs() << "Drivetrain Right: "
                << m_drivetrain.GetRightPosition().to<double>() << "\n";
    wpi::outs() << "Drivetrain Gyro: " << m_drivetrain.GetAngle().to<double>()
                << "\n";
}

void Robot::AutonomousPeriodic() { TeleopPeriodic(); }

void Robot::TeleopPeriodic() {
    for (int i = 2; i <= 12; i++) {
        if (m_driveStick1.GetRawButtonPressed(i)) {
            ButtonPacket message{"DriveStick1", i, true};
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
        case State::kIdle: {
            if (m_appendageStick2.GetRawButtonPressed(1)) {
                m_vision.TurnLEDOn();
                m_flywheel.Shoot();
                m_state = State::kStartFlywheel;
            }
            break;
        }
        // Allow the flywheel to spin up to the correct angular velocity.
        case State::kStartFlywheel: {
            if (m_flywheel.AtGoal()) {
                m_timer.Reset();
                m_timer.Start();
                m_state = State::kStartConveyor;
            }
            break;
        }
        // Feed balls until conveyor is empty and timeout has occurred.
        case State::kStartConveyor: {
            if (m_timer.HasElapsed(5_s) && !m_intake.IsUpperSensorBlocked()) {
                m_flywheel.SetGoal(0_rad_per_s);
                m_vision.TurnLEDOff();
                m_timer.Stop();
                m_state = State::kIdle;
            }
            break;
        }
    }

    auto& ds = frc::DriverStation::GetInstance();
    HIDPacket message{"",
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
    Publish(message);
}

}  // namespace frc3512

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<frc3512::Robot>(); }
#endif
