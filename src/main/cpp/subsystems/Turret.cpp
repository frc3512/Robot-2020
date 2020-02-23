// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Turret.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Turret;

Turret::Turret() : PublishNode("Turret") {
    m_motor.Set(0);
#ifndef RUNNING_FRC_TESTS
    m_encoder.SetDistancePerRotation(kDpP);
#else
    m_encoder.SetDistancePerPulse(kDpP);
#endif
    Reset();
    m_lastAngle = units::radian_t{m_encoder.GetDistance()};
}

void Turret::SetVoltage(units::volt_t voltage) { m_motor.SetVoltage(voltage); }

void Turret::ResetEncoder() { m_encoder.Reset(); }

void Turret::Reset() {
    ResetEncoder();
    m_controller.Reset();
}

bool Turret::GetLeftHallTriggered() const { return !m_leftHall.Get(); }

bool Turret::GetRightHallTriggered() const { return !m_rightHall.Get(); }

units::radian_t Turret::GetAngle() {
    return units::radian_t{m_encoder.GetDistance()};
}

void Turret::Enable() {
    m_lastTime = std::chrono::steady_clock::now();
    m_controller.Enable();
    m_thread.StartPeriodic(5_ms);
}

void Turret::Disable() {
    m_controller.Disable();
    m_thread.Stop();
}

void Turret::Iterate() {
    auto now = std::chrono::steady_clock::now();
    m_controller.SetMeasuredOutputs(GetAngle());
    m_controller.SetHardLimitOutputs(GetLeftHallTriggered(),
                                     GetRightHallTriggered());
    m_controller.Update(now - m_lastTime, now.time_since_epoch());

    // Set motor input
    SetVoltage(m_controller.ControllerVoltage());

    auto nextPose = m_controller.GetNextPose();
    TurretPosePacket message{"TurretPose",
                             nextPose.Translation().X().to<double>(),
                             nextPose.Translation().Y().to<double>(),
                             nextPose.Rotation().Radians().to<double>()};
    Publish(message);

    m_lastTime = now;
}

void Turret::ProcessMessage(const CommandPacket& message) {
    if (message.topic == "Robot/TeleopInit" && !message.reply) {
        Enable();
    } else if (message.topic == "Robot/AutonomousInit" && !message.reply) {
        Enable();
    } else if (message.topic == "Robot/DisabledInit" && !message.reply) {
        Disable();
    }
}
