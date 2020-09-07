// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Turret.hpp"

#include "subsystems/Drivetrain.hpp"
#include "subsystems/Vision.hpp"

using namespace frc3512;

Turret::Turret(Vision& vision, Drivetrain& drivetrain)
    : m_vision(vision), m_drivetrain(drivetrain) {
#ifndef RUNNING_FRC_TESTS
    m_encoder.SetDistancePerRotation(TurretController::kDpR);
#else
    m_encoder.SetDistancePerPulse(TurretController::kDpR);
#endif
    Reset();
}

void Turret::SetDirection(Direction direction) {
    if (m_manualOverride) {
        if (direction == Direction::kCW && !HasPassedCWLimit()) {
            SetVoltage(-4.0_V);
        } else if (direction == Direction::kCCW && !HasPassedCCWLimit()) {
            SetVoltage(4.0_V);
        } else {
            SetVoltage(0.0_V);
        }
    }
}

void Turret::SetVoltage(units::volt_t voltage) { m_motor.SetVoltage(voltage); }

void Turret::SetManualOverride() { m_manualOverride = true; }

void Turret::ResetEncoder() { m_encoder.Reset(); }

void Turret::Reset() { m_controller.Reset(); }

units::radian_t Turret::GetAngle() const {
    return units::radian_t{-m_encoder.GetDistance()} + kOffset;
}

bool Turret::HasPassedCCWLimit() const {
    return GetAngle() > (wpi::math::pi * 1_rad / 2.0);
}

bool Turret::HasPassedCWLimit() const {
    return GetAngle() < -(wpi::math::pi * 1_rad / 2.0);
}

frc::Pose2d Turret::GetNextPose() const { return m_controller.GetNextPose(); }

void Turret::EnableController() {
    m_lastTime = std::chrono::steady_clock::now();
    m_controller.Enable();
}

void Turret::DisableController() { m_controller.Disable(); }

void Turret::ControllerPeriodic() {
    auto now = std::chrono::steady_clock::now();
    m_controller.SetMeasuredOutputs(GetAngle());

    m_controller.SetDrivetrainStatus(m_drivetrain.GetNextXhat());
    m_controller.SetHardLimitOutputs(HasPassedCWLimit(), HasPassedCWLimit());
    m_controller.Update(now - m_lastTime, now - GetStartTime());

    auto globalMeasurement = m_vision.GetGlobalMeasurement();
    if (globalMeasurement) {
        auto turretInGlobal = globalMeasurement.value();

        frc::Transform2d turretInGlobalToDrivetrainInGlobal{
            frc::Pose2d{
                TurretController::kDrivetrainToTurretFrame.Translation(),
                TurretController::kDrivetrainToTurretFrame.Rotation()
                        .Radians() +
                    GetAngle()},
            frc::Pose2d{}};
        auto drivetrainInGlobal =
            turretInGlobal.pose.TransformBy(turretInGlobalToDrivetrainInGlobal);

        m_drivetrain.CorrectWithGlobalOutputs(
            drivetrainInGlobal.Translation().X(),
            drivetrainInGlobal.Translation().Y(),
            globalMeasurement.value().timestamp);
    }

    // Set motor input
    if (!m_manualOverride) {
        auto u = m_controller.GetInputs();
        SetVoltage(units::volt_t{u(TurretController::Input::kVoltage)});
    }

    m_lastTime = now;
}
