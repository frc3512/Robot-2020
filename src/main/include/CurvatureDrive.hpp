// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <tuple>

namespace frc3512 {

/**
 * Returns 0.0 if the given value is within the specified range around zero. The
 * remaining range between the deadband and 1.0 is scaled from 0.0 to 1.0.
 *
 * @param value    Value to clip.
 * @param deadband Range around zero.
 */
double ApplyDeadband(double number, double deadband);

/**
 * Curvature drive method for differential drive platform.
 *
 * The rotation argument controls the curvature of the robot's path rather than
 * its rate of heading change. This makes the robot more controllable at high
 * speeds. Constant-curvature turning can be overridden for turn-in-place
 * maneuvers.
 *
 * @param xSpeed           The robot's speed along the X axis [-1.0..1.0].
 *                         Forward is positive.
 * @param zRotation        The robot's rotation rate around the Z axis
 *                         [-1.0..1.0]. Clockwise is positive.
 * @param allowTurnInPlace If set, overrides constant-curvature turning for
 *                         turn-in-place maneuvers.
 */
std::tuple<double, double> CurvatureDrive(double xSpeed, double zRotation,
                                          bool allowTurnInPlace);

}  // namespace frc3512
