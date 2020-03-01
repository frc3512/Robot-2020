// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/SubsystemBase.hpp"

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
