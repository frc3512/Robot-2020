// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <drake/math/discrete_algebraic_riccati_equation.h>
#include <frc/system/Discretization.h>
#include <units/time.h>

#include "controllers/DARE.hpp"

namespace frc3512 {

/**
 * Returns the LQR controller gain for the given coefficients and plant.
 *
 * @param A  Continuous system matrix of the plant being controlled.
 * @param B  Continuous input matrix of the plant being controlled.
 * @param Q  The state cost matrix.
 * @param R  The input cost matrix.
 * @param dt Discretization timestep.
 */
template <int States, int Inputs>
Eigen::Matrix<double, Inputs, States> LQR(
    const Eigen::Matrix<double, States, States>& A,
    const Eigen::Matrix<double, States, Inputs>& B,
    const Eigen::Matrix<double, States, States>& Q,
    const Eigen::Matrix<double, Inputs, Inputs>& R, units::second_t dt) {
    Eigen::Matrix<double, States, States> discA;
    Eigen::Matrix<double, States, Inputs> discB;
    frc::DiscretizeAB<States, Inputs>(A, B, dt, &discA, &discB);

    Eigen::Matrix<double, States, States> S =
        drake::math::DiscreteAlgebraicRiccatiEquation(discA, discB, Q, R);
    return (discB.transpose() * S * discB + R)
        .llt()
        .solve(discB.transpose() * S * discA);
}

/**
 * Returns the LQR controller gain for the given coefficients and plant.
 *
 * @param A Discrete system matrix of the plant being controlled.
 * @param B Discrete input matrix of the plant being controlled.
 * @param Q The state cost matrix.
 * @param R The input cost matrix.
 * @param N The state-input cross-term cost matrix.
 */
template <int States, int Inputs>
Eigen::Matrix<double, Inputs, States> LQR(
    const Eigen::Matrix<double, States, States>& A,
    const Eigen::Matrix<double, States, Inputs>& B,
    const Eigen::Matrix<double, States, States>& Q,
    const Eigen::Matrix<double, Inputs, Inputs>& R,
    const Eigen::Matrix<double, States, Inputs>& N) {
    Eigen::Matrix<double, States, States> S =
        DARE<States, Inputs>(A, B, Q, R, N);
    return (B.transpose() * S * B + R)
        .llt()
        .solve(B.transpose() * S * A + N.transpose());
}

}  // namespace frc3512
