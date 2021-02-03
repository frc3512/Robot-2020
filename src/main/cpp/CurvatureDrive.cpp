// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include "CurvatureDrive.hpp"

#include <algorithm>
#include <cmath>

namespace frc3512 {

double ApplyDeadband(double value, double deadband) {
    if (std::abs(value) > deadband) {
        if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
        } else {
            return (value + deadband) / (1.0 - deadband);
        }
    } else {
        return 0.0;
    }
}

std::tuple<double, double> CurvatureDrive(double xSpeed, double zRotation,
                                          bool allowTurnInPlace) {
    xSpeed = std::clamp(xSpeed, -1.0, 1.0);
    zRotation = std::clamp(zRotation, -1.0, 1.0);

    double leftSpeed = 0.0;
    double rightSpeed = 0.0;

    if (allowTurnInPlace) {
        leftSpeed = xSpeed + zRotation;
        rightSpeed = xSpeed - zRotation;
    } else {
        leftSpeed = xSpeed + std::abs(xSpeed) * zRotation;
        rightSpeed = xSpeed - std::abs(xSpeed) * zRotation;
    }

    // Normalize wheel speeds
    double maxMagnitude = std::max(std::abs(leftSpeed), std::abs(rightSpeed));
    if (maxMagnitude > 1.0) {
        leftSpeed /= maxMagnitude;
        rightSpeed /= maxMagnitude;
    }

    return {leftSpeed, rightSpeed};
}

}  // namespace frc3512
