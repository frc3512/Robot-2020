// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Turret.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Turret;

Turret::Turret() {
    m_motor.Set(0);
    ResetEncoder();
}

void Turret::SetVoltage(units::volt_t voltage) { m_motor.SetVoltage(voltage); }

void Turret::ResetEncoder() { m_encoder.Reset(); }

bool Turret::GetLeftHallTriggered() const { return !m_leftHall.Get(); }

bool Turret::GetRightHallTriggered() const { return !m_rightHall.Get(); }
