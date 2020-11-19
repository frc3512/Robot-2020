// Copyright (c) 2016-2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Flywheel.hpp"

#include <frc/RobotBase.h>
#include <frc/RobotController.h>

#include "CANSparkMaxUtil.hpp"
#include "controllers/TurretController.hpp"
#include "subsystems/Drivetrain.hpp"

using namespace frc3512;

Flywheel::Flywheel(Drivetrain& drivetrain) : m_drivetrain(drivetrain) {
    SetCANSparkMaxBusUsage(m_leftGrbx, Usage::kMinimal);
    SetCANSparkMaxBusUsage(m_rightGrbx, Usage::kMinimal);

    // Ensures CANSparkMax::Get() returns an initialized value
    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);

    m_encoder.SetDistancePerPulse(FlywheelController::kDpP);
    m_encoder.SetSamplesToAverage(5);
    m_leftGrbx.SetInverted(false);
    m_rightGrbx.SetInverted(false);

    // TODO: add more entries to the look up table
    m_table.Insert(125_in, 450_rad_per_s);
    m_table.Insert(200_in, 510_rad_per_s);
    m_table.Insert(268_in, 525_rad_per_s);
    m_table.Insert(312_in, 550_rad_per_s);
    m_table.Insert(326_in, 650_rad_per_s);

    Reset();
    SetGoal(0_rad_per_s);
}

void Flywheel::SetVoltage(units::volt_t voltage) {
    m_leftGrbx.SetVoltage(voltage);
    m_rightGrbx.SetVoltage(-voltage);
}

units::radian_t Flywheel::GetAngle() {
    return units::radian_t{m_encoder.GetDistance()};
}

units::radians_per_second_t Flywheel::GetAngularVelocity() const {
    return m_angularVelocity;
}

void Flywheel::SetGoal(units::radians_per_second_t velocity) {
    m_controller.SetGoal(velocity);
}

units::radians_per_second_t Flywheel::GetGoal() const {
    return m_controller.GetGoal();
}

bool Flywheel::AtGoal() { return m_controller.AtGoal(); }

void Flywheel::Shoot() { SetGoal(GetReferenceForPose(m_drivetrain.GetPose())); }

bool Flywheel::IsOn() const { return GetGoal() > 0_rad_per_s; }

bool Flywheel::IsReady() { return GetGoal() > 0_rad_per_s && AtGoal(); }

void Flywheel::Reset() {
    m_controller.Reset();
    m_encoder.Reset();
    m_angle = GetAngle();
    m_lastAngle = m_angle;
}

units::ampere_t Flywheel::GetCurrentDraw() const {
    return m_flywheelSim.GetCurrentDraw();
}

units::radians_per_second_t Flywheel::GetReferenceForPose(
    const frc::Pose2d& drivetrainPose) const {
    auto turretPose =
        TurretController::DrivetrainToTurretInGlobal(drivetrainPose);
    return m_table[turretPose.Translation().Distance(
        kTargetPoseInGlobal.Translation())];
}

void Flywheel::RobotPeriodic() {
    using State = FlywheelController::State;

    m_angularVelocityRefEntry.SetDouble(
        m_controller.GetReferences()(State::kAngularVelocity));
    m_angularVelocityStateEntry.SetDouble(
        m_controller.GetStates()(State::kAngularVelocity));
    m_isOnEntry.SetBoolean(IsOn());
    m_isReadyEntry.SetBoolean(IsReady());
    m_controllerEnabledEntry.SetBoolean(m_controller.IsEnabled());
}

void Flywheel::ControllerPeriodic() {
    m_angle = GetAngle();
    m_time = frc2::Timer::GetFPGATimestamp();

    // WPILib uses the time between pulses in GetRate() to calculate velocity,
    // but this is very noisy for high-resolution encoders. Instead, we
    // calculate a velocity from the change in angle over change in time, which
    // is more precise.
    m_angularVelocity = m_velocityFilter.Calculate((m_angle - m_lastAngle) /
                                                   (m_time - m_lastTime));

    Eigen::Matrix<double, 1, 1> y;
    y << GetAngularVelocity().to<double>();
    Eigen::Matrix<double, 1, 1> u = m_controller.UpdateAndLog(y);
    SetVoltage(units::volt_t{u(0)});

    if constexpr (frc::RobotBase::IsSimulation()) {
        m_flywheelSim.SetInput(frc::MakeMatrix<1, 1>(
            m_leftGrbx.Get() * frc::RobotController::GetInputVoltage()));
        m_flywheelPositionSim.SetInput(frc::MakeMatrix<1, 1>(
            m_leftGrbx.Get() * frc::RobotController::GetInputVoltage()));

        m_flywheelSim.Update(m_controller.GetDt());
        m_flywheelPositionSim.Update(m_controller.GetDt());

        m_encoderSim.SetDistance(m_flywheelPositionSim.GetOutput(0));
    }

    m_lastAngle = m_angle;
    m_lastTime = m_time;
}
