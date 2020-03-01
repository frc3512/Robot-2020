// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Turret.hpp"

using namespace frc3512;

Turret::Turret() : SubsystemBase("Turret") {
    m_motor.Set(0);
#ifndef RUNNING_FRC_TESTS
    m_encoder.SetDistancePerRotation(TurretController::kDpR);
#else
    m_encoder.SetDistancePerPulse(TurretController::kDpR);
#endif
    m_lastAngle = units::radian_t{m_encoder.GetDistance()};
}

void Turret::SetVoltage(units::volt_t voltage) { m_motor.SetVoltage(voltage); }

void Turret::ResetEncoder() { m_encoder.Reset(); }

void Turret::Reset() { m_controller.Reset(); }

bool Turret::GetLeftHallTriggered() const { return !m_leftHall.Get(); }

bool Turret::GetRightHallTriggered() const { return !m_rightHall.Get(); }

units::radian_t Turret::GetAngle() {
    return units::radian_t{-m_encoder.GetDistance()};
}

void Turret::EnableController() {
    m_lastTime = std::chrono::steady_clock::now();
    m_controller.Enable();
    m_thread.StartPeriodic(5_ms);
}

void Turret::DisableController() {
    m_controller.Disable();
    m_thread.Stop();
}

void Turret::Iterate() {
    auto now = std::chrono::steady_clock::now();
    m_controller.SetMeasuredOutputs(GetAngle());
    m_controller.SetHardLimitOutputs(GetLeftHallTriggered(),
                                     GetRightHallTriggered());
    m_controller.Update(now - m_lastTime, now - m_startTime);

    // Set motor input
    SetVoltage(m_controller.ControllerVoltage());

    m_lastTime = now;
}
