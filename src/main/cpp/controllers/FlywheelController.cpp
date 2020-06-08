// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "controllers/FlywheelController.hpp"

#include <frc/RobotController.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/units.h>

using namespace frc3512;
using namespace frc3512::Constants;

FlywheelController::FlywheelController() {
    m_y.setZero();
    Reset();
}

void FlywheelController::Enable() {
    m_lqr.Enable();
    m_ff.Enable();
    m_isEnabled = true;
}

void FlywheelController::Disable() {
    m_lqr.Disable();
    m_ff.Disable();
    m_isEnabled = false;
}

bool FlywheelController::IsEnabled() const { return m_isEnabled; }

void FlywheelController::SetGoal(units::radians_per_second_t angularVelocity) {
    m_nextR << angularVelocity.to<double>();
}

units::radians_per_second_t FlywheelController::GetGoal() const {
    return units::radians_per_second_t(m_nextR(0));
}

bool FlywheelController::AtGoal() const { return m_atGoal; }

void FlywheelController::SetMeasuredAngularVelocity(
    units::radians_per_second_t angularVelocity) {
    m_y << angularVelocity.to<double>();
}

const Eigen::Matrix<double, 1, 1>& FlywheelController::GetReferences() const {
    return m_r;
}

const Eigen::Matrix<double, 1, 1>& FlywheelController::GetStates() const {
    return m_observer.Xhat();
}

const Eigen::Matrix<double, 1, 1>& FlywheelController::GetInputs() const {
    return m_u;
}

const Eigen::Matrix<double, 1, 1>& FlywheelController::GetOutputs() const {
    return m_y;
}

void FlywheelController::Update(units::second_t dt,
                                units::second_t elapsedTime) {
    velocityLogger.Log(elapsedTime, m_y(Output::kAngularVelocity),
                       m_observer.Xhat(State::kAngularVelocity),
                       m_nextR(State::kAngularVelocity));
    voltageLogger.Log(elapsedTime, m_u(Input::kVoltage),
                      frc::RobotController::GetInputVoltage());

    m_observer.Correct(m_u, m_y);

    m_lqr.Update(m_observer.Xhat().block<1, 1>(0, 0),
                 m_nextR.block<1, 1>(0, 0));

    // To conserve battery when the flywheel doesn't have to be spinning, don't
    // apply a negative voltage to slow down.
    if (m_nextR(0) == 0.0) {
        m_u(0) = 0.0;
    } else {
        m_u = (m_lqr.U() + m_ff.Calculate(m_nextR)) * 12.0 /
              frc::RobotController::GetInputVoltage();
    }

    ScaleCapU(&m_u);

    m_atGoal = units::math::abs(units::radians_per_second_t{
                   m_r(State::kAngularVelocity) -
                   m_observer.Xhat(State::kAngularVelocity)}) <
               kAngularVelocityTolerance;
    m_r = m_nextR;
    m_observer.Predict(m_u * frc::RobotController::GetInputVoltage() / 12.0,
                       dt);
}

void FlywheelController::ScaleCapU(Eigen::Matrix<double, 1, 1>* u) {
    bool outputCapped = std::abs((*u)(0)) > 12.0;

    if (outputCapped) {
        *u *= 12.0 / u->lpNorm<Eigen::Infinity>();
    }
}

void FlywheelController::Reset() {
    m_observer.Reset();
    m_r.setZero();
    m_nextR.setZero();
    m_u.setZero();
}
