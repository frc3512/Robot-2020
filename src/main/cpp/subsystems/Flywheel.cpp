// Copyright (c) 2016-2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Flywheel.hpp"

#include <frc/RobotController.h>

using namespace frc3512;
using namespace std::chrono_literals;

Flywheel::Flywheel() : PublishNode("Flywheel") {
    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);
    m_encoder.SetDistancePerPulse(FlywheelController::kDpP);
    m_encoder.SetSamplesToAverage(5);
    m_rightGrbx.SetInverted(false);
    m_leftGrbx.SetInverted(false);
    SetGoal(0_rad_per_s);
    Reset();

    // TODO: add more entries to the look up table
    m_table.insert(0_m, 5_rad_per_s);
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
    if (velocity > 0_rad_per_s) {
        m_timer.Reset();
        m_timer.Start();
    }
}

units::radians_per_second_t Flywheel::GetGoal() const {
    return m_controller.AngularVelocityGoal();
}

bool Flywheel::AtGoal() const {
    // TODO: Re-add line when encoder is installed on flywheel.
    // return m_controller.AtGoal();
    return m_timer.HasElapsed(5_s);
}

void Flywheel::Shoot() {
    std::scoped_lock lock(m_controllerMutex);
    // TODO: Put LUT back
    /*    auto angularVelocity =
            m_table.linear_interp(m_nextTurretPose.Translation().Distance(
                kTargetPoseInGlobal.Translation()));
        SetGoal(angularVelocity);
    */
    SetGoal(8_V / 12_V * FlywheelController::kMaxAngularVelocity);
}

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

void Flywheel::ProcessMessage(const CommandPacket& message) {
    if (message.topic == "Robot/TeleopInit" && !message.reply) {
        Enable();
    } else if (message.topic == "Robot/AutonomousInit" && !message.reply) {
        Enable();
    } else if (message.topic == "Robot/DisabledInit" && !message.reply) {
        Disable();
    }
}

void Flywheel::ProcessMessage(const TurretPosePacket& message) {
    m_nextTurretPose =
        frc::Pose2d(units::meter_t{message.x}, units::meter_t{message.y},
                    units::radian_t{message.heading});
}
