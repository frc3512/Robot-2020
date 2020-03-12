// Copyright (c) 2016-2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Flywheel.hpp"

#include <frc/RobotController.h>

#include "subsystems/Turret.hpp"

using namespace frc3512;
using namespace std::chrono_literals;

Flywheel::Flywheel(Turret& turret)
    : ControllerSubsystemBase("Flywheel"), m_turret(turret) {
    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);
    m_encoder.SetDistancePerPulse(FlywheelController::kDpP);
    m_encoder.SetSamplesToAverage(5);
    m_rightGrbx.SetInverted(false);
    m_leftGrbx.SetInverted(false);
    SetGoal(0_rad_per_s);
    Reset();

    // TODO: add more entries to the look up table
    m_table.insert(125_in, 450_rad_per_s);
    m_table.insert(200_in, 510_rad_per_s);
    m_table.insert(268_in, 525_rad_per_s);
    m_table.insert(312_in, 550_rad_per_s);
    m_table.insert(326_in, 650_rad_per_s);
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

void Flywheel::EnableController() {
    m_lastTime = std::chrono::steady_clock::now();
    m_controller.Enable();
}

void Flywheel::DisableController() { m_controller.Disable(); }

void Flywheel::SetGoal(units::radians_per_second_t velocity) {
    m_timer.Reset();
    m_timer.Start();
    m_controller.SetGoal(velocity);
}

units::radians_per_second_t Flywheel::GetGoal() const {
    auto ref = m_controller.GetReferences();
    return units::radians_per_second_t{
        ref(FlywheelController::State::kAngularVelocity, 0)};
}

bool Flywheel::AtGoal() {
    bool atGoal = m_controller.AtGoal() || m_timer.HasElapsed(3_s);
    if (atGoal) {
        m_timer.Stop();
    }
    return atGoal;
}

void Flywheel::Shoot() {
    std::scoped_lock lock(m_controllerMutex);
    auto turretPose = m_turret.GetNextPose();
    auto angularVelocity = m_table.linear_interp(
        turretPose.Translation().Distance(kTargetPoseInGlobal.Translation()));
    SetGoal(angularVelocity);
}

bool Flywheel::IsShooting() const {
    std::scoped_lock lock(m_controllerMutex);
    return GetGoal() > 0_rad_per_s;
}

void Flywheel::Reset() {
    m_controller.Reset();
    m_encoder.Reset();
}

void Flywheel::ControllerPeriodic() {
    auto now = std::chrono::steady_clock::now();
    m_controller.SetMeasuredAngularVelocity(GetAngularVelocity());
    m_controller.Update(now - m_lastTime, now - GetStartTime());
    // Set motor input
    auto u = m_controller.GetInputs();
    SetVoltage(units::volt_t{u(FlywheelController::Input::kVoltage, 0)});
    m_lastTime = now;
}
