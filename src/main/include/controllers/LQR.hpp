// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Eigen/Core>
#include <Eigen/src/Cholesky/LLT.h>
#include <drake/math/discrete_algebraic_riccati_equation.h>
#include <frc/system/Discretization.h>
#include <units/time.h>

namespace frc3512 {

/**
 * Returns the LQR controller gain for the given coefficients and plant.
 *
 * @param A      Continuous system matrix of the plant being controlled.
 * @param B      Continuous input matrix of the plant being controlled.
 * @param Q      The state cost matrix.
 * @param R      The input cost matrix.
 * @param dt     Discretization timestep.
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

}  // namespace frc3512
