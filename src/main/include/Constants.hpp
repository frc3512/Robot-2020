// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <units/time.h>

/**
 * This namespace is for robot-wide constants that aren't related to hardware
 * configuration. See HWConfig.hpp for hardware configuration.
 */
namespace frc3512::Constants {

// Joystick axis deadband range
constexpr double kJoystickDeadband = 0.05;

constexpr bool kAtHomeChallenge = true;

constexpr units::second_t kControllerPeriod = 5_ms;

}  // namespace frc3512::Constants
