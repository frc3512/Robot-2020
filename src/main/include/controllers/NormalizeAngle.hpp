// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <wpi/math>

/**
 * Constrains theta to within the range (-pi, pi].
 *
 * @param theta Angle to normalize
 */
static constexpr double NormalizeAngle(double theta) {
    // Constrain theta to within (-3pi, pi)
    const int n_pi_pos = (theta + wpi::math::pi) / 2.0 / wpi::math::pi;
    theta -= n_pi_pos * 2.0 * wpi::math::pi;

    // Cut off the bottom half of the above range to constrain within
    // (-pi, pi]
    const int n_pi_neg = (theta - wpi::math::pi) / 2.0 / wpi::math::pi;
    theta -= n_pi_neg * 2.0 * wpi::math::pi;

    return theta;
}
