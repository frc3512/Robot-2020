// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/ControlledSubsystemBase.hpp"

#include "Constants.hpp"

using namespace frc3512;

wpi::SmallVector<ControlledSubsystemBase*, 8>
    ControlledSubsystemBase::m_controllers;

frc::Notifier ControlledSubsystemBase::m_notifier{[] {
    for (auto& controller : m_controllers) {
        controller->ControllerPeriodic();
    }
}};

ControlledSubsystemBase::ControlledSubsystemBase() {
    Disable();
    m_controllers.emplace_back(this);
}

ControlledSubsystemBase::~ControlledSubsystemBase() {
    Disable();
    m_controllers.erase(
        std::remove(m_controllers.begin(), m_controllers.end(), this),
        m_controllers.end());
    Enable();
}

void ControlledSubsystemBase::Enable() {
    m_notifier.StartPeriodic(frc3512::Constants::kDt);
}

void ControlledSubsystemBase::Disable() { m_notifier.Stop(); }
