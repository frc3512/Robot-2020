// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Drivetrain.hpp"

#include <cmath>
#include <limits>
#include <string>

#include <frc/RobotController.h>

using namespace frc3512;
using namespace frc3512::Constants::Robot;

Drivetrain::Drivetrain() : PublishNode("Drivetrain") {
    m_drive.SetDeadband(kJoystickDeadband);

    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);

    m_leftGrbx.SetInverted(true);
    m_drive.SetRightSideInverted(false);

    m_leftEncoder.SetSamplesToAverage(10);
    m_rightEncoder.SetSamplesToAverage(10);

    m_leftEncoder.SetReverseDirection(false);
    m_rightEncoder.SetReverseDirection(true);
    m_leftEncoder.SetDistancePerPulse(DrivetrainController::kDpP);
    m_rightEncoder.SetDistancePerPulse(DrivetrainController::kDpP);
}

void Drivetrain::Drive(double throttle, double turn, bool isQuickTurn) {
    std::lock_guard lock(m_motorControllerMutex);
    m_drive.CurvatureDrive(throttle, turn, isQuickTurn);
}

void Drivetrain::SetLeftManual(double value) { m_leftGrbx.Set(value); }

void Drivetrain::SetRightManual(double value) { m_rightGrbx.Set(value); }

units::radian_t Drivetrain::GetAngle() const {
    return units::degree_t{-m_gyro.GetAngle()};
}

units::radians_per_second_t Drivetrain::GetAngularRate() const {
    return units::degrees_per_second_t{-m_gyro.GetRate()};
}

void Drivetrain::ResetGyro() { m_gyro.Reset(); }

void Drivetrain::CalibrateGyro() { m_gyro.Calibrate(); }

units::meter_t Drivetrain::GetLeftPosition() const {
    return units::meter_t{m_leftEncoder.GetDistance()};
}

units::meter_t Drivetrain::GetRightPosition() const {
    return units::meter_t{m_rightEncoder.GetDistance()};
}

units::meters_per_second_t Drivetrain::GetLeftVelocity() const {
    return units::meters_per_second_t{m_leftEncoder.GetRate()};
}

units::meters_per_second_t Drivetrain::GetRightVelocity() const {
    return units::meters_per_second_t{m_rightEncoder.GetRate()};
}

void Drivetrain::ResetEncoders() {
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
}

void Drivetrain::Reset(const frc::Pose2d& initialPose) {
    m_controller.Reset(initialPose);
    ResetEncoders();
    ResetGyro();
}

void Drivetrain::EnableController() {
    m_lastTime = std::chrono::steady_clock::now();
    m_controllerThread.StartPeriodic(Constants::kDt);
    m_controller.Enable();
    m_drive.SetSafetyEnabled(false);
}

void Drivetrain::DisableController() {
    m_controllerThread.Stop();
    m_controller.Disable();
    m_drive.SetSafetyEnabled(true);
}

bool Drivetrain::IsControllerEnabled() const {
    return m_controller.IsEnabled();
}

void Drivetrain::Iterate() {
    m_controller.SetMeasuredInputs(
        units::volt_t{m_leftGrbx.Get() *
                      frc::RobotController::GetInputVoltage()},
        units::volt_t{m_rightGrbx.Get() *
                      frc::RobotController::GetInputVoltage()});
    m_controller.SetMeasuredLocalOutputs(GetAngle(), GetLeftPosition(),
                                         GetRightPosition());
    auto now = std::chrono::steady_clock::now();
    m_controller.Update(now - m_lastTime, now - m_startTime);

    if (!m_controller.IsOpenLoop()) {
        // Set motor inputs
        auto u = m_controller.GetInputs();
        SetLeftManual(u(0, 0) / 12.0);
        SetRightManual(u(1, 0) / 12.0);
    }
    m_lastTime = now;
}

void Drivetrain::SetWaypoints(const std::vector<frc::Pose2d>& waypoints) {
    m_controller.SetWaypoints(waypoints);
}

bool Drivetrain::AtGoal() const { return m_controller.AtGoal(); }

void Drivetrain::ProcessMessage(const CommandPacket& message) {
    if (message.topic == "Robot/DisabledInit" && !message.reply) {
        DisableController();
    }
    if (message.topic == "Robot/AutonomousInit" && !message.reply) {
        Reset();
        EnableController();
        m_controller.SetOpenLoop(false);
        m_startTime = std::chrono::steady_clock::now();
    }
    if (message.topic == "Robot/TeleopInit" && !message.reply) {
        m_startTime = std::chrono::steady_clock::now();
        EnableController();
        m_controller.SetOpenLoop(true);
        EnablePeriodic();
    }
}

void Drivetrain::ProcessMessage(const HIDPacket& message) {
    if (GetRawButton(message, 0, 1)) {
        Drive(-message.y1 * 0.5, message.x2 * 0.5, GetRawButton(message, 1, 2));
    } else {
        Drive(-message.y1, message.x2, GetRawButton(message, 1, 2));
    }
}
