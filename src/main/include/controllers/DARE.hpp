// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <drake/math/discrete_algebraic_riccati_equation.h>

namespace frc3512 {

/**
 * Returns solution to the DARE.
 *
 * @param A System matrix.
 * @param B Input matrix.
 * @param Q State cost matrix.
 * @param R Input cost matrix.
 * @param N State-input cross-term cost matrix.
 */
template <int States, int Inputs>
Eigen::Matrix<double, States, States> DARE(
    const Eigen::Matrix<double, States, States>& A,
    const Eigen::Matrix<double, States, Inputs>& B,
    const Eigen::Matrix<double, States, States>& Q,
    const Eigen::Matrix<double, Inputs, Inputs>& R,
    const Eigen::Matrix<double, States, Inputs>& N) {
    Eigen::Matrix<double, States, States> scrA =
        A - B * R.llt().solve(N.transpose());
    Eigen::Matrix<double, States, States> scrQ =
        Q - N * R.llt().solve(N.transpose());

    return drake::math::DiscreteAlgebraicRiccatiEquation(scrA, B, scrQ, R);
}

}  // namespace frc3512
