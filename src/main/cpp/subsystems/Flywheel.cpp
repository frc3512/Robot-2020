// Copyright (c) 2016-2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Flywheel.hpp"

#include <frc/RobotController.h>

using namespace frc3512;
using namespace frc3512::Constants::Flywheel;
using namespace std::chrono_literals;

Flywheel::Flywheel() : PublishNode("Flywheel") {
    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);
    m_encoder.SetDistancePerPulse(kDpP);
    m_encoder.SetSamplesToAverage(5);
    m_rightGrbx.SetInverted(false);
    m_leftGrbx.SetInverted(false);
    SetGoal(0_rad_per_s);
    Reset();
}

void Flywheel::SetVoltage(units::volt_t voltage) {
    m_leftGrbx.SetVoltage(voltage);
    m_rightGrbx.SetVoltage(-voltage);
}

units::radian_t Flywheel::GetAngle() {
    return units::radian_t{m_encoder.GetDistance()};
}

units::radians_per_second_t Flywheel::GetAngularVelocity() {
    return units::radians_per_second_t{m_encoder.GetRate()};
}

void Flywheel::Enable() {
    m_lastTime = std::chrono::steady_clock::now();
    m_controller.Enable();
    m_thread.StartPeriodic(5_ms);
}

void Flywheel::Disable() {
    m_controller.Disable();
    m_thread.Stop();
}

void Flywheel::SetGoal(units::radians_per_second_t velocity) {
    m_controller.SetGoal(velocity);
}

bool Flywheel::AtGoal() { return m_controller.AtGoal(); }

void Flywheel::Iterate() {
    auto now = std::chrono::steady_clock::now();
    m_controller.SetMeasuredAngularVelocity(GetAngularVelocity());
    m_controller.Update(now - m_lastTime, now.time_since_epoch());
    // Set motor input
    SetVoltage(m_controller.ControllerVoltage());
    m_lastTime = now;
}

void Flywheel::Reset() {
    m_controller.Reset();
    m_encoder.Reset();
}

void Flywheel::ProcessMessage(const ButtonPacket& message) {
    if (message.topic == "Robot/DriveStick1" && message.button == 5 &&
        message.pressed) {
        SetGoal(450.0_rad_per_s);  // 4297.18 RPM
    } else if (message.topic == "Robot/DriveStick1" && message.button == 3 &&
               message.pressed) {
        SetGoal(250.0_rad_per_s);  // 4774.65 RPM
    } else if (message.topic == "Robot/DriveStick1" && message.button == 6 &&
               message.pressed) {
        SetGoal(550.0_rad_per_s);  // 5252.11 RPM
    } else if (message.topic == "Robot/DriveStick1" && message.button == 4 &&
               message.pressed) {
        SetGoal(600.0_rad_per_s);  // 5729.58 RPM
    } else if (message.topic == "Robot/DriveStick1" && message.button == 7 &&
               message.pressed) {
        SetGoal(650.0_rad_per_s);  // 6207.04 RPM
    } else if (message.topic == "Robot/DriveStick1" && message.button == 8 &&
               message.pressed) {
        SetGoal(700.0_rad_per_s);  // 6684.51 RPM
    } else if (message.topic == "Robot/DriveStick1" && message.button == 9 &&
               message.pressed) {
        SetGoal(750.0_rad_per_s);  // 7161.97 RPM
    } else if (message.topic == "Robot/DriveStick1" && message.button == 10 &&
               message.pressed) {
        SetGoal(800.0_rad_per_s);  // 7639.44 RPM
    } else if (message.topic == "Robot/DriveStick1" && message.button == 11 &&
               message.pressed) {
        SetGoal(850.0_rad_per_s);  // 8116.90 RPM
    } else if (message.topic == "Robot/DriveStick1" && message.button == 12 &&
               message.pressed) {
        SetGoal(900.0_rad_per_s);  // 8594.37 RPM
    } else if (message.topic == "Robot/DriveStick1" && message.button == 2 &&
               message.pressed) {
        SetGoal(0_rad_per_s);  // 0 RPM
    }
}

void Flywheel::ProcessMessage(const CommandPacket& message) {
    if (message.topic == "Robot/TeleopInit" && !message.reply) {
        Enable();
    } else if (message.topic == "Robot/AutonomousInit" && !message.reply) {
        Enable();
    } else if (message.topic == "Robot/DisabledInit" && !message.reply) {
        Disable();
    }
}
