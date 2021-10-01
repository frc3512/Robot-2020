// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/time.h>

/**
 * This namespace is for robot-wide constants that aren't related to hardware
 * configuration. See HWConfig.hpp for hardware configuration.
 */
namespace frc3512::Constants {

/// Joystick axis deadband range
constexpr double kJoystickDeadband = 0.05;

/**
 * Whether we're building for the 2021 Infinite Recharge At-Home Challenge
 * (IR\@H).
 *
 * This enables IR\@H-specific autonomous modes and flywheel speed presets.
 */
constexpr bool kAtHomeChallenge = false;

/// The period at which feedback controllers run
constexpr units::second_t kControllerPeriod = 5_ms;

}  // namespace frc3512::Constants
