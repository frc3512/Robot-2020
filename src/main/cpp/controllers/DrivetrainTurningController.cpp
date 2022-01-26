// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "controllers/DrivetrainTurningController.hpp"

#include <algorithm>
#include <cmath>

#include <frc/MathUtil.h>
#include <frc/fmt/Eigen.h>
#include <frc/system/plant/LinearSystemId.h>

using namespace frc3512;

const frc::LinearSystem<2, 2, 2> DrivetrainTurningController::kPlant{
    GetPlant()};

DrivetrainTurningController::DrivetrainTurningController() {}

void DrivetrainTurningController::SetHeadingGoal(units::radian_t newHeading) {
    m_headingGoal = frc::AngleModulus(newHeading);
}

bool DrivetrainTurningController::HaveHeadingGoal() const {
    units::radian_t ref{m_r(State::kHeading)};
    return m_headingGoal != ref;
}

void DrivetrainTurningController::AbortTurnInPlace() {
    units::radian_t ref{m_r(State::kHeading)};
    m_headingGoal = ref;
}

bool DrivetrainTurningController::AtHeading() const {
    units::radian_t ref{m_r(State::kHeading)};
    return (ref >= m_headingGoal + 0.1_rad) &&
           (ref <= m_headingGoal - 0.1_rad) && m_atReferences;
}

Eigen::Vector<double, 2> DrivetrainTurningController::Calculate(
    const Eigen::Vector<double, 7>& x) {
    m_u = Eigen::Vector<double, 2>::Zero();

    if (HaveHeadingGoal()) {
        Eigen::Vector<double, 3> xHeading = {x[State::kHeading],
                                             x[State::kLeftVelocity],
                                             x[State::kRightVelocity]};

        frc::TrapezoidProfile<units::radian>::State references = {
            units::radian_t{m_nextR(State::kHeading)},
            units::radians_per_second_t{0}};
        frc::TrapezoidProfile<units::radian> profile{
            m_constraints, m_headingState, references};

        auto profiledReference =
            profile.Calculate(Constants::kControllerPeriod);

        Eigen::Vector<double, 3> m_nextRHeading = {
            profiledReference.position(), m_nextR[State::kLeftVelocity],
            m_nextR[State::kRightVelocity]};

        m_u = m_turningLQR.Calculate(xHeading, m_nextRHeading) +
              m_turningFF.Calculate(m_nextRHeading);
        m_u = frc::DesaturateInputVector<2>(m_u, 12.0);

        Eigen::Vector<double, 5> error =
            m_nextR.block<5, 1>(0, 0) - x.block<5, 1>(0, 0);
        error(State::kHeading) =
            frc::AngleModulus(units::radian_t{error(State::kHeading)}).value();
        UpdateAtReferences(error);

        m_r = m_nextR;
    }

    if (AtHeading() && HaveHeadingGoal()) {
        units::radian_t ref{m_r[State::kHeading]};
        m_headingGoal = ref;
    }

    return m_u;
}

frc::LinearSystem<2, 2, 2> DrivetrainTurningController::GetPlant() {
    return frc::LinearSystemId::IdentifyDrivetrainSystem(kLinearV, kLinearA,
                                                         kAngularV, kAngularA);
}

frc::LinearSystem<3, 2, 3> DrivetrainTurningController::TurningDynamics() {
    Eigen::Matrix<double, 3, 2> B;
    B.block<2, 2>(1, 0) = kPlant.B();
    Eigen::Matrix<double, 3, 3> A;
    A.block<2, 2>(1, 1) = kPlant.A();
    Eigen::Matrix<double, 3, 3> C;
    C.setIdentity();
    Eigen::Matrix<double, 3, 2> D;
    D.setZero();

    return frc::LinearSystem<3, 2, 3>{A, B, C, D};
}

void DrivetrainTurningController::UpdateAtReferences(
    const Eigen::Vector<double, 5>& error) {
    m_atReferences = std::abs(error(0)) < kPositionTolerance &&
                     std::abs(error(1)) < kPositionTolerance &&
                     std::abs(error(2)) < kAngleTolerance &&
                     std::abs(error(3)) < kVelocityTolerance &&
                     std::abs(error(4)) < kVelocityTolerance;
}
