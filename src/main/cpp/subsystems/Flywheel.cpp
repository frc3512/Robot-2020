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
}

void Flywheel::Shoot() {
    std::scoped_lock lock(m_poseDataMutex);
    auto angularVelocity =
        m_table.linear_interp(DistanceToTarget(m_nextTurretPose));
    SetGoal(angularVelocity);
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

units::meter_t Flywheel::DistanceToTarget(const frc::Pose2d& nextPose) {
    return units::math::sqrt(units::math::pow<2, units::meter_t>(
                                 targetPoseInGlobal.Translation().Y() -
                                 m_nextTurretPose.Translation().Y()) +
                             units::math::pow<2, units::meter_t>(
                                 targetPoseInGlobal.Translation().X() -
                                 m_nextTurretPose.Translation().X()));
}
