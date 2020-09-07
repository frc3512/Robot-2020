// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/ControllerSubsystemBase.hpp"

using namespace frc3512;

wpi::SmallVector<ControllerSubsystemBase*, 16>
    ControllerSubsystemBase::m_controllers;
frc::Notifier ControllerSubsystemBase::m_notifier{
    Constants::kControllerPrio, &ControllerSubsystemBase::RunControllers};
std::chrono::steady_clock::time_point ControllerSubsystemBase::m_startTime{
    std::chrono::steady_clock::now()};

ControllerSubsystemBase::ControllerSubsystemBase() {
    Disable();
    m_controllers.emplace_back(this);
}

ControllerSubsystemBase::~ControllerSubsystemBase() {
    Disable();
    m_controllers.erase(
        std::remove(m_controllers.begin(), m_controllers.end(), this),
        m_controllers.end());
    Enable();
}

void ControllerSubsystemBase::Enable() {
    m_startTime = std::chrono::steady_clock::now();
    m_notifier.StartPeriodic(frc3512::Constants::kDt);
}

void ControllerSubsystemBase::Disable() { m_notifier.Stop(); }

std::chrono::steady_clock::time_point ControllerSubsystemBase::GetStartTime() {
    return m_startTime;
}

void ControllerSubsystemBase::RunControllers() {
    for (auto& controller : m_controllers) {
        controller->ControllerPeriodic();
    }
}
