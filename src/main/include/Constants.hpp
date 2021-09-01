// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

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

// Joystick axis deadband range
constexpr double kJoystickDeadband = 0.05;

constexpr bool kAtHomeChallenge = true;

constexpr units::second_t kControllerPeriod = 5_ms;

namespace Vision {
// Camera name
constexpr char kCameraName[] = "gloworm";

// Camera height
constexpr units::meter_t kCameraHeight = 39_in;

// Camera pitch
constexpr units::degree_t kCameraPitch = 22.8_deg;

// Diagonal pi camera V1 FOV
constexpr units::degree_t kCameraDiagonalFOV = 74.8_deg;
}  // namespace Vision

}  // namespace frc3512::Constants
