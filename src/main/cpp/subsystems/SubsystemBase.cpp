// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/SubsystemBase.hpp"

#include <frc/DriverStation.h>

using namespace frc3512;

wpi::SmallVector<SubsystemBase*, 16> SubsystemBase::m_subsystems;

void SubsystemBase::RunAllDisabledInit() {
    for (auto& subsystem : m_subsystems) {
        subsystem->DisabledInit();
    }
}

void SubsystemBase::RunAllAutonomousInit() {
    for (auto& subsystem : m_subsystems) {
        subsystem->AutonomousInit();
    }
}

void SubsystemBase::RunAllTeleopInit() {
    // Consumes button edge events produced in disabled mode
    auto& ds = frc::DriverStation::GetInstance();
    for (int stick = 0; stick < frc::DriverStation::kJoystickPorts; ++stick) {
        for (int button = 1; button < 32; ++button) {
            ds.GetStickButtonPressed(stick, button);
            ds.GetStickButtonReleased(stick, button);
        }
    }

    for (auto& subsystem : m_subsystems) {
        subsystem->TeleopInit();
    }
}

void SubsystemBase::RunAllRobotPeriodic() {
    for (auto& subsystem : m_subsystems) {
        subsystem->RobotPeriodic();
    }
}

void SubsystemBase::RunAllDisabledPeriodic() {
    for (auto& subsystem : m_subsystems) {
        subsystem->DisabledPeriodic();
    }
}

void SubsystemBase::RunAllAutonomousPeriodic() {
    for (auto& subsystem : m_subsystems) {
        subsystem->AutonomousPeriodic();
    }
}

void SubsystemBase::RunAllTeleopPeriodic() {
    for (auto& subsystem : m_subsystems) {
        subsystem->TeleopPeriodic();
    }
}
