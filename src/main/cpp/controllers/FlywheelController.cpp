// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "controllers/FlywheelController.hpp"

#include <frc/RobotController.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/units.h>

using namespace frc3512;
using namespace frc3512::Constants;
using namespace frc3512::Constants::Flywheel;

FlywheelController::FlywheelController(const std::array<double, 1>& Qelems,
                                       const std::array<double, 1>& Relems,
                                       units::second_t dt)
    : m_lqr{m_plant, Qelems, Relems, dt} {
    m_y.setZero();
    Reset();
}

void FlywheelController::Enable() {
    m_lqr.Enable();
    m_isEnabled = true;
}

void FlywheelController::Disable() {
    m_lqr.Disable();
    m_isEnabled = false;
}

bool FlywheelController::IsEnabled() const { return m_isEnabled; }

void FlywheelController::SetGoal(units::radians_per_second_t angularVelocity) {
    m_nextR << angularVelocity.to<double>();
}

bool FlywheelController::AtGoal() { return m_atGoal; }

void FlywheelController::SetMeasuredAngularVelocity(
    units::radians_per_second_t angularVelocity) {
    m_y << angularVelocity.to<double>();
}

units::radians_per_second_t FlywheelController::EstimatedAngularVelocity()
    const {
    return units::radians_per_second_t{m_observer.Xhat(0)};
}

units::radians_per_second_t FlywheelController::AngularVelocityError() const {
    return units::radians_per_second_t{m_r(0, 0) - m_observer.Xhat(0)};
}

units::radians_per_second_t FlywheelController::AngularVelocityGoal() const {
    return units::radians_per_second_t{m_nextR(0, 0)};
}

units::volt_t FlywheelController::ControllerVoltage() const {
    return units::volt_t{m_u(0, 0)};
}

units::volt_t FlywheelController::ControllerVoltageError() const {
    return units::volt_t{0};
}

void FlywheelController::Update(units::second_t dt,
                                units::second_t elapsedTime) {
    velocityLogger.Log(elapsedTime, m_y(0, 0),
                       EstimatedAngularVelocity().to<double>(),
                       AngularVelocityGoal().to<double>());
    voltageLogger.Log(elapsedTime, ControllerVoltage().to<double>(),
                      ControllerVoltageError().to<double>(),
                      frc::RobotController::GetInputVoltage());

    m_observer.Correct(m_u, m_y);

    m_lqr.Update(m_observer.Xhat().block<1, 1>(0, 0),
                 m_nextR.block<1, 1>(0, 0));
    m_u = m_lqr.U();

    ScaleCapU(&m_u);

    m_atGoal =
        units::math::abs(AngularVelocityError()) < kAngularVelocityTolerance;
    m_r = m_nextR;
    m_observer.Predict(m_u * frc::RobotController::GetInputVoltage() / 12.0,
                       dt);
}

const frc::LinearSystem<2, 1, 1>& FlywheelController::GetAugmentedPlant()
    const {
    return m_augmentedPlant;
}

void FlywheelController::ScaleCapU(Eigen::Matrix<double, 1, 1>* u) {
    bool outputCapped = std::abs((*u)(0, 0)) > 12.0;

    if (outputCapped) {
        *u *= 12.0 / u->lpNorm<Eigen::Infinity>();
    }
}

void FlywheelController::Reset() {
    m_observer.Reset();
    m_r.setZero();
    m_nextR.setZero();
}
