// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "subsystems/SubsystemBase.hpp"

#include <frc/DriverStation.h>

using namespace frc3512;

wpi::SmallVector<SubsystemBase*, 8> SubsystemBase::m_subsystems;

SubsystemBase::SubsystemBase() { m_subsystems.emplace_back(this); }

SubsystemBase::~SubsystemBase() {
    m_subsystems.erase(
        std::remove(m_subsystems.begin(), m_subsystems.end(), this));
}

void SubsystemBase::RunAllSimulationInit() {
    for (auto& subsystem : m_subsystems) {
        subsystem->SimulationInit();
    }
}

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
    ConsumeButtonEdgeEvents();

    for (auto& subsystem : m_subsystems) {
        subsystem->TeleopInit();
    }
}

void SubsystemBase::RunAllTestInit() {
    ConsumeButtonEdgeEvents();

    for (auto& subsystem : m_subsystems) {
        subsystem->TestInit();
    }
}

void SubsystemBase::RunAllRobotPeriodic() {
    for (auto& subsystem : m_subsystems) {
        subsystem->RobotPeriodic();
    }
}

void SubsystemBase::RunAllSimulationPeriodic() {
    for (auto& subsystem : m_subsystems) {
        subsystem->SimulationPeriodic();
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

void SubsystemBase::RunAllTestPeriodic() {
    for (auto& subsystem : m_subsystems) {
        subsystem->TestPeriodic();
    }
}

void SubsystemBase::ConsumeButtonEdgeEvents() {
    // Consumes button edge events produced in disabled mode
    for (int stick = 0; stick < frc::DriverStation::kJoystickPorts; ++stick) {
        for (int button = 1; button < 32; ++button) {
            frc::DriverStation::GetStickButtonPressed(stick, button);
            frc::DriverStation::GetStickButtonReleased(stick, button);
        }
    }
}
