// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <units/units.h>
#include <wpi/math>

namespace frc3512::Constants {

namespace Robot {
/*
 * Joystick and buttons
 */

// Joystick Ports
constexpr int kDriveStick1Port = 0;
constexpr int kDriveStick2Port = 1;
constexpr int kAppendageStickPort = 2;
constexpr int kAppendageStick2Port = 3;

// Joystick axis deadband range
constexpr double kJoystickDeadband = 0.02;
}  // namespace Robot

namespace Drivetrain {

// Motor Ports
constexpr int kLeftMasterPort = 16;
constexpr int kLeftSlavePort = 1;
constexpr int kRightMasterPort = 14;
constexpr int kRightSlavePort = 15;

// Encoder Ports
constexpr int kLeftEncoderA = 0;
constexpr int kLeftEncoderB = 1;
constexpr int kRightEncoderA = 2;
constexpr int kRightEncoderB = 3;

// Distance per Pulse
constexpr units::meter_t kWheelRadius = 3_in;
constexpr double kDpP =
    (2.0 * wpi::math::pi * kWheelRadius.to<double>()) / 2048.0;

}  // namespace Drivetrain

namespace Flywheel {
constexpr int kLeftPort = 9;
constexpr int kRightPort = 10;
constexpr int kEncoderA = 6;
constexpr int kEncoderB = 7;

constexpr auto kV = 0.00957_V / 1_rad_per_s;
constexpr auto kA = 0.02206 / 1_rad_per_s / 1.0_s;
constexpr auto kMaxAngularVelocity = 1000.5_rad_per_s;
constexpr units::radians_per_second_t kAngularVelocityTolerance = 7.0_rad_per_s;

constexpr double kGearRatio = 2.0;
constexpr double kDpP = (wpi::math::pi * 2.0) / 512.0;
}  // namespace Flywheel

constexpr auto kDt = 0.00505_s;
constexpr int kControllerPrio = 50;
}  // namespace frc3512::Constants
