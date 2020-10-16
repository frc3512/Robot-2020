// Copyright (c) 2016-2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Flywheel.hpp"

#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>

#include "CANSparkMaxUtil.hpp"
#include "subsystems/Turret.hpp"

using namespace frc3512;

Flywheel::Flywheel(Turret& turret) : m_turret(turret) {
    SetCANSparkMaxBusUsage(m_leftGrbx, Usage::kMinimal);
    SetCANSparkMaxBusUsage(m_rightGrbx, Usage::kMinimal);

    m_encoder.SetDistancePerPulse(FlywheelController::kDpP);
    m_encoder.SetSamplesToAverage(5);
    m_rightGrbx.SetInverted(false);
    m_leftGrbx.SetInverted(false);
    SetGoal(0_rad_per_s);
    Reset();

    // TODO: add more entries to the look up table
    m_table.Insert(125_in, 450_rad_per_s);
    m_table.Insert(200_in, 510_rad_per_s);
    m_table.Insert(268_in, 525_rad_per_s);
    m_table.Insert(312_in, 550_rad_per_s);
    m_table.Insert(326_in, 650_rad_per_s);
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

void Flywheel::EnableController() { m_controller.Enable(); }

void Flywheel::DisableController() { m_controller.Disable(); }

void Flywheel::SetGoal(units::radians_per_second_t velocity) {
    m_timer.Reset();
    m_timer.Start();
    m_controller.SetGoal(velocity);
}

units::radians_per_second_t Flywheel::GetGoal() const {
    return m_controller.GetGoal();
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
    auto angularVelocity = m_table[turretPose.Translation().Distance(
        kTargetPoseInGlobal.Translation())];
    SetGoal(angularVelocity);
}

bool Flywheel::IsOn() const {
    std::scoped_lock lock(m_controllerMutex);
    return GetGoal() > 0_rad_per_s;
}

bool Flywheel::IsReady() {
    std::scoped_lock lock(m_controllerMutex);
    return GetGoal() > 0_rad_per_s && AtGoal();
}

void Flywheel::Reset() {
    m_controller.Reset();
    m_encoder.Reset();
}

void Flywheel::RobotPeriodic() {
    m_encoderEntry.SetDouble(GetAngle().to<double>());
    m_goalEntry.SetDouble(GetGoal().to<double>());
    m_isOnEntry.SetBoolean(IsOn());
    m_isReadyEntry.SetBoolean(IsReady());
    m_controllerEnabledEntry.SetBoolean(m_controller.IsEnabled());
}

void Flywheel::ControllerPeriodic() {
    Eigen::Matrix<double, 1, 1> y;
    y << GetAngularVelocity().to<double>();
    Eigen::Matrix<double, 1, 1> u = m_controller.UpdateAndLog(y);
    SetVoltage(units::volt_t{u(0)});

    if constexpr (frc::RobotBase::IsSimulation()) {
        m_flywheelSim.SetInput(frc::MakeMatrix<1, 1>(
            m_leftGrbx.Get() * frc::RobotController::GetInputVoltage()));

        m_flywheelSim.Update(Constants::kDt);

        m_encoderSim.SetRate(m_flywheelSim.GetAngularVelocity().to<double>());

        frc::sim::RoboRioSim::SetVInVoltage(
            frc::sim::BatterySim::Calculate({m_flywheelSim.GetCurrentDraw()})
                .to<double>());
    }
}
