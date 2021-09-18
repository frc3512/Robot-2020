// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "controllers/FlywheelController.hpp"

#include <frc/RobotController.h>
#include <frc/system/plant/LinearSystemId.h>

using namespace frc3512;

FlywheelController::FlywheelController() { Reset(); }

void FlywheelController::SetGoal(units::radians_per_second_t angularVelocity) {
    if (m_nextR(0) == angularVelocity.value()) {
        return;
    }

    m_nextR = Eigen::Vector<double, 1>{angularVelocity.value()};
    m_atGoal = false;
}

units::radians_per_second_t FlywheelController::GetGoal() const {
    return units::radians_per_second_t(m_nextR(0));
}

bool FlywheelController::AtGoal() const { return m_atGoal; }

void FlywheelController::Reset() {
    m_r.setZero();
    m_nextR.setZero();
}

Eigen::Vector<double, 1> FlywheelController::Calculate(
    const Eigen::Vector<double, 1>& x) {
    // To conserve battery when the flywheel doesn't have to be spinning, don't
    // apply a negative voltage to slow down.
    if (m_nextR(0) == 0.0) {
        m_u = Eigen::Vector<double, 1>::Zero();
    } else {
        m_u = m_lqr.Calculate(x, m_r) + m_ff.Calculate(m_nextR) +
              Eigen::Vector<double, 1>{kS.value()};
    }

    m_u = frc::DesaturateInputVector<1>(m_u, 12.0);

    // m_nextR is used here so AtGoal() returns false after calling SetGoal()
    UpdateAtGoal(units::radians_per_second_t{(m_nextR - x)(0)});
    m_r = m_nextR;

    return m_u;
}

frc::LinearSystem<1, 1, 1> FlywheelController::GetPlant() {
    return frc::LinearSystemId::IdentifyVelocitySystem<units::radian>(kV, kA);
}

void FlywheelController::UpdateAtGoal(units::radians_per_second_t error) {
    // Add hysteresis to AtGoal() so it won't chatter due to measurement noise
    // when the angular velocity drops. The threshold when going out of
    // tolerance (e.g., down after shooting a ball) is lower than the threshold
    // when going into tolerance (e.g., up from recovery).
    if (m_atGoal && error > kAngularVelocityShotThreshold) {
        m_atGoal = false;
    } else if (!m_atGoal && error < kAngularVelocityRecoveryThreshold) {
        m_atGoal = true;
    }
}
