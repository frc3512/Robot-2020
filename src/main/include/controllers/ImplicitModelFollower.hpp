// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <algorithm>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <drake/math/discrete_algebraic_riccati_equation.h>
#include <frc/StateSpaceUtil.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/system/Discretization.h>
#include <frc/system/LinearSystem.h>
#include <units/time.h>
#include <wpi/array.h>

namespace frc3512 {

/**
 * Contains the controller coefficients and logic for an implicit model
 * follower.
 *
 * Implicit model following lets us design a feedback controller that erases the
 * dynamics of our system and makes it behave like some other system. This can
 * be used to make a drivetrain more controllable during teleop driving by
 * making it behave like a slower or more benign drivetrain.
 *
 * For more on the underlying math, read appendix C.3 in
 * https://file.tavsys.net/control/controls-engineering-in-frc.pdf.
 */
template <int States, int Inputs>
class ImplicitModelFollower {
public:
    /**
     * Constructs a controller with the given coefficients and plant.
     *
     * @param plant    The plant being controlled.
     * @param plantRef The plant whose dynamics should be followed.
     * @param Qelems   The maximum desired error tolerance for each state.
     * @param Relems   The maximum desired control effort for each input.
     * @param dt       Discretization timestep.
     */
    template <int Outputs>
    ImplicitModelFollower(
        const frc::LinearSystem<States, Inputs, Outputs>& plant,
        const frc::LinearSystem<States, Inputs, Outputs>& plantRef,
        const wpi::array<double, States>& Qelems,
        const wpi::array<double, Inputs>& Relems, units::second_t dt)
        : ImplicitModelFollower<States, Inputs>(plant.A(), plant.B(),
                                                plantRef.A(), plantRef.B(),
                                                Qelems, Relems, dt) {}

    /**
     * Constructs a controller with the given coefficients and plant.
     *
     * @param A      Continuous system matrix of the plant being controlled.
     * @param B      Continuous input matrix of the plant being controlled.
     * @param Aref   Continuous system matrix whose dynamics should be followed.
     * @param Bref   Continuous input matrix whose dynamics should be followed.
     * @param Qelems The maximum desired error tolerance for each state.
     * @param Relems The maximum desired control effort for each input.
     * @param dt     Discretization timestep.
     */
    ImplicitModelFollower(const Eigen::Matrix<double, States, States>& A,
                          const Eigen::Matrix<double, States, Inputs>& B,
                          const Eigen::Matrix<double, States, States>& Aref,
                          const Eigen::Matrix<double, States, States>& Bref,
                          const wpi::array<double, States>& Qelems,
                          const wpi::array<double, Inputs>& Relems,
                          units::second_t dt) {
        // Discretize real dynamics
        Eigen::Matrix<double, States, States> discA;
        Eigen::Matrix<double, States, Inputs> discB;
        frc::DiscretizeAB<States, Inputs>(A, B, dt, &discA, &discB);

        // Discretize desired dynamics
        Eigen::Matrix<double, States, States> discAref;
        Eigen::Matrix<double, States, Inputs> discBref;
        frc::DiscretizeAB<States, Inputs>(Aref, Bref, dt, &discAref, &discBref);

        // Find initial Q and R weights
        Eigen::Matrix<double, States, States> Q = frc::MakeCostMatrix(Qelems);
        Eigen::Matrix<double, Inputs, Inputs> R = frc::MakeCostMatrix(Relems);

        Eigen::Matrix<double, States, States> Adiff = discA - discAref;

        Eigen::Matrix<double, States, States> Qimf =
            Adiff.transpose() * Q * Adiff;
        Eigen::Matrix<double, Inputs, Inputs> Rimf =
            discB.transpose() * Q * discB + R;
        Eigen::Matrix<double, States, Inputs> Nimf =
            Adiff.transpose() * Q * discB;

        // Nudge eigenvalues of Qimf slightly more positive if Q <= 0, since
        // this is usually caused by numerical imprecision
        Eigen::SelfAdjointEigenSolver<decltype(Qimf)> Qeigen{Qimf};
        if (!std::all_of(Qeigen.eigenvalues().data(),
                         Qeigen.eigenvalues().data() + States,
                         [](const auto& elem) { return elem >= 0.0; })) {
            Qimf += decltype(Qimf)::Identity() * 1e-10;
        }

        Eigen::Matrix<double, States, States> S =
            drake::math::DiscreteAlgebraicRiccatiEquation(discA, discB, Qimf,
                                                          Rimf, Nimf);

        // K = (BᵀSB + R)⁻¹(BᵀSA + Nᵀ)
        m_K = (discB.transpose() * S * discB + Rimf)
                  .llt()
                  .solve(discB.transpose() * S * discA + Nimf.transpose());

        // Find u_imf that makes real model match reference model.
        //
        // x_k+1 = Ax_k + Bu_imf
        // z_k+1 = Aref z_k + Bref u_k
        //
        // Let x_k = z_k.
        //
        // x_k+1 = z_k+1
        // Ax_k + Bu_imf = Aref x_k + Bref u_k
        // Bu_imf = Aref x_k - Ax_k + Bref u_k
        // Bu_imf = (Aref - A)x_k + Bref u_k
        // u_imf = B^+ ((Aref - A)x_k + Bref u_k)
        // u_imf = -B^+ (A - Aref)x_k + B^+ Bref u_k

        // The first term makes the open-loop poles that of the reference
        // system, and the second term makes the input behave like that of the
        // reference system.
        m_B = discB.householderQr().solve(discBref);

        Reset();
    }

    /**
     * Returns the controller matrix K.
     */
    const Eigen::Matrix<double, Inputs, States>& K() const { return m_K; }

    /**
     * Returns an element of the controller matrix K.
     *
     * @param i Row of K.
     * @param j Column of K.
     */
    double K(int i, int j) const { return m_K(i, j); }

    /**
     * Returns the control input vector u.
     *
     * @return The control input.
     */
    const Eigen::Vector<double, Inputs>& U() const { return m_u; }

    /**
     * Returns an element of the control input vector u.
     *
     * @param i Row of u.
     *
     * @return The row of the control input vector.
     */
    double U(int i) const { return m_u(i, 0); }

    /**
     * Resets the controller.
     */
    void Reset() { m_u.setZero(); }

    /**
     * Returns the next output of the controller.
     *
     * @param x The current state x.
     * @param u The current input for the original model.
     */
    Eigen::Vector<double, Inputs> Calculate(
        const Eigen::Vector<double, States>& x,
        const Eigen::Vector<double, Inputs>& u) {
        m_u = -m_K * x + m_B * u;
        return m_u;
    }

private:
    // Computed controller output
    Eigen::Vector<double, Inputs> m_u;

    // Controller gain
    Eigen::Matrix<double, Inputs, States> m_K;

    // Input space conversion gain
    Eigen::Matrix<double, Inputs, Inputs> m_B;
};

}  // namespace frc3512
