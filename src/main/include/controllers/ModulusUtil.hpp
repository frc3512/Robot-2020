// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <cmath>
#include <type_traits>

#include <units/units.h>
#include <wpi/math>

/**
 * Returns modulus of error where error is the difference between the reference
 * and a measurement.
 *
 * This implements modular subtraction defined as:
 *
 * e = (r mod m - x mod m) mod m
 *
 * with an offset in the modulus range for minimum input.
 *
 * @param reference Reference input of a controller.
 * @param measurement The current measurement.
 * @param minimumInput The minimum value expected from the input.
 * @param maximumInput The maximum value expected from the input.
 */
template <typename T>
T GetModulusError(T reference, T measurement, T minimumInput, T maximumInput) {
    T modulus = maximumInput - minimumInput;

    if constexpr (std::is_same_v<T, double>) {
        T error =
            std::fmod(reference, modulus) - std::fmod(measurement, modulus);

        // Moduli on the difference arguments establish a precondition for the
        // following modulus.
        return std::fmod(error - minimumInput, modulus) + minimumInput;
    } else if constexpr (std::is_same_v<T, int>) {
        T error = reference % modulus - measurement % modulus;

        // Moduli on the difference arguments establish a precondition for the
        // following modulus.
        return (error - minimumInput) % modulus + minimumInput;
    } else {
        T error = units::math::fmod(reference, modulus) -
                  units::math::fmod(measurement, modulus);

        // Moduli on the difference arguments establish a precondition for the
        // following modulus.
        return units::math::fmod(error - minimumInput, modulus) + minimumInput;
    }
}

/**
 * Constrains theta to within the range (-pi, pi].
 *
 * @param theta Angle to normalize
 */
template <typename T>
constexpr T NormalizeAngle(T theta) {
    // Constrain theta to within (-3pi, pi)
    const int n_pi_pos = (theta + T{wpi::math::pi}) / 2.0 / wpi::math::pi;
    theta -= T{n_pi_pos * 2.0 * wpi::math::pi};

    // Cut off the bottom half of the above range to constrain within
    // (-pi, pi]
    const int n_pi_neg = (theta - T{wpi::math::pi}) / 2.0 / wpi::math::pi;
    theta -= T{n_pi_neg * 2.0 * wpi::math::pi};

    return theta;
}
