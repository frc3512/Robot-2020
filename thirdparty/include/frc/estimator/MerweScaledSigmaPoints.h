/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <cmath>

#include <Eigen/Cholesky>
#include <Eigen/Core>

namespace frc {

/**
 * Generates sigma points and weights according to Van der Merwe's 2004
 * dissertation[1] for the UnscentedKalmanFilter class.
 *
 * It parametizes the sigma points using alpha, beta, kappa terms, and is the
 * version seen in most publications. Unless you know better, this should be
 * your default choice.
 *
 * @tparam States The dimensionality of the state. 2*States+1 weights will be
 *                generated.
 *
 * [1] R. Van der Merwe "Sigma-Point Kalman Filters for Probabilitic
 *     Inference in Dynamic State-Space Models" (Doctoral dissertation)
 */
template <int States>
class MerweScaledSigmaPoints {
 public:
  /**
   * Constructs a generator for Van der Merwe scaled sigma points.
   *
   * @param alpha Determines the spread of the sigma points around the mean.
   *              Usually a small positive value (1e-3).
   * @param beta Incorporates prior knowledge of the distribution of the mean.
   *             For Gaussian distributions, beta = 2 is optimal.
   * @param kappa Secondary scaling parameter usually set to 0 or 3 - States.
   */
  MerweScaledSigmaPoints(double alpha = 1e-3, double beta = 2,
                         int kappa = 3 - States) {
    m_alpha = alpha;
    m_kappa = kappa;

    ComputeWeights(beta);
  }

  /**
   * Returns number of sigma points for each variable in the state x.
   */
  int NumSigmas() { return 2 * States + 1; }

  /**
   * Computes the sigma points for an unscented Kalman filter given the mean
   * (x) and covariance(P) of the filter.
   *
   * @param x An array of the means.
   * @param P Covariance of the filter.
   *
   * @return Two dimensional array of sigma points. Each column contains all of
   *         the sigmas for one dimension in the problem space. Ordered by
   *         Xi_0, Xi_{1..n}, Xi_{n+1..2n}.
   *
   */
  Eigen::Matrix<double, 2 * States + 1, States> SigmaPoints(
      const Eigen::Matrix<double, 1, States>& x,
      const Eigen::Matrix<double, States, States>& P) {
    double lambda = std::pow(m_alpha, 2) * (States + m_kappa) - States;
    Eigen::Matrix<double, States, States> U =
        ((lambda + States) * P).llt().matrixU();

    Eigen::Matrix<double, 2 * States + 1, States> sigmas;
    sigmas.template block<1, States>(0, 0) = x;
    for (int k = 0; k < States; ++k) {
      sigmas.template block<1, States>(k + 1, 0) =
          x + U.template block<1, States>(k, 0);
      sigmas.template block<1, States>(States + k + 1, 0) =
          x - U.template block<1, States>(k, 0);
    }

    return sigmas;
  }

  /**
   * Returns the weight for each sigma point for the mean.
   */
  const Eigen::Matrix<double, 1, 2 * States + 1>& Wm() const { return m_Wm; }

  /**
   * Returns an element of the weight for each sigma point for the mean.
   *
   * @param i Element of vector to return.
   */
  double Wm(int i) const { return m_Wm(0, i); }

  /**
   * Returns the weight for each sigma point for the covariance.
   */
  const Eigen::Matrix<double, 1, 2 * States + 1>& Wc() const { return m_Wc; }

  /**
   * Returns an element of the weight for each sigma point for the covariance.
   *
   * @param i Element of vector to return.
   */
  double Wc(int i) const { return m_Wc(0, i); }

 private:
  Eigen::Matrix<double, 1, 2 * States + 1> m_Wm;
  Eigen::Matrix<double, 1, 2 * States + 1> m_Wc;
  double m_alpha;
  int m_kappa;

  /**
   * Computes the weights for the scaled unscented Kalman filter.
   *
   * @param beta Incorporates prior knowledge of the distribution of the mean.
   */
  void ComputeWeights(double beta) {
    double lambda = std::pow(m_alpha, 2) * (States + m_kappa) - States;

    double c = 0.5 / (States + lambda);
    m_Wm = Eigen::Matrix<double, 1, 2 * States + 1>::Constant(c);
    m_Wc = Eigen::Matrix<double, 1, 2 * States + 1>::Constant(c);

    m_Wm(0, 0) = lambda / (States + lambda);
    m_Wc(0, 0) = lambda / (States + lambda) + (1 - std::pow(m_alpha, 2) + beta);
  }
};

}  // namespace frc
