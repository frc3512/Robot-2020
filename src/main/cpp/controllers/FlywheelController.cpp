// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include "controllers/FlywheelController.hpp"

#include <frc/RobotController.h>
#include <frc/system/plant/LinearSystemId.h>

using namespace frc3512;
using namespace frc3512::Constants;

FlywheelController::FlywheelController()
    : ControllerBase("Flywheel", {ControllerLabel{"Angular velocity", "rad/s"}},
                     {ControllerLabel{"Voltage", "V"}},
                     {ControllerLabel{"Angular velocity", "rad/s"}}) {
    Reset();
}

void FlywheelController::SetGoal(units::radians_per_second_t angularVelocity) {
    m_nextR << angularVelocity.to<double>();
    auto absError = units::math::abs(
        units::radians_per_second_t{m_nextR(State::kAngularVelocity) -
                                    m_observer.Xhat(State::kAngularVelocity)});

    // Add hysteresis to AtGoal() so it won't chatter due to measurement noise
    // when the angular velocity drops. Threshold when going out of tolerance
    // (e.g., down after shooting a ball) is tolerance + 20. Threshold when
    // going into tolerance (e.g., up from recovery) is threshold.
    if (m_atGoal && absError > kAngularVelocityTolerance + 20_rad_per_s) {
        m_atGoal = false;
    } else if (!m_atGoal && absError < kAngularVelocityTolerance) {
        m_atGoal = true;
    }
}

units::radians_per_second_t FlywheelController::GetGoal() const {
    return units::radians_per_second_t(m_nextR(0));
}

bool FlywheelController::AtGoal() const { return m_atGoal; }

const Eigen::Matrix<double, 1, 1>& FlywheelController::GetReferences() const {
    return m_lqr.R();
}

const Eigen::Matrix<double, 1, 1>& FlywheelController::GetStates() const {
    return m_observer.Xhat();
}

void FlywheelController::Reset() {
    m_observer.Reset();
    m_r.setZero();
    m_nextR.setZero();
}

Eigen::Matrix<double, 1, 1> FlywheelController::Update(
    const Eigen::Matrix<double, 1, 1>& y, units::second_t dt) {
    m_observer.Correct(GetInputs(), y);

    Eigen::Matrix<double, 1, 1> u;

    // To conserve battery when the flywheel doesn't have to be spinning, don't
    // apply a negative voltage to slow down.
    if (m_nextR(0) == 0.0 || !IsEnabled()) {
        m_lqr.Calculate(m_observer.Xhat(), m_r);
        u(0) = 0.0;
    } else {
        u = m_lqr.Calculate(m_observer.Xhat(), m_r) + m_ff.Calculate(m_nextR);
    }

    u *= 12.0 / frc::RobotController::GetInputVoltage();
    u = frc::NormalizeInputVector<1>(u, 12.0);

    m_atGoal = units::math::abs(units::radians_per_second_t{
                   m_lqr.R(State::kAngularVelocity) -
                   m_observer.Xhat(State::kAngularVelocity)}) <
               kAngularVelocityTolerance;
    m_r = m_nextR;
    m_observer.Predict(u * frc::RobotController::GetInputVoltage() / 12.0, dt);

    return u;
}

const frc::LinearSystem<1, 1, 1>& FlywheelController::GetPlant() const {
    return m_plant;
}
