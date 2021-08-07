// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <units/time.h>

namespace frc3512::Constants {

// Joystick axis deadband range
constexpr double kJoystickDeadband = 0.05;

constexpr bool kAtHomeChallenge = true;

constexpr units::second_t kControllerPeriod = 5_ms;

}  // namespace frc3512::Constants
