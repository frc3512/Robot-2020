// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <units/units.h>
#include <wpi/math>

namespace frc3512::Constants {
/**
 * Controller Base
 * NOTE: Default values are used here.
 */
constexpr int kControllerPrio = 50;
constexpr auto kDt = 0.00505_s;

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
constexpr int kLeftMasterPort = 0;
constexpr int kLeftSlavePort = 1;
constexpr int kRightMasterPort = 2;
constexpr int kRightSlavePort = 3;

// Encoder Ports
constexpr int kLeftEncoderA = 2;
constexpr int kLeftEncoderB = 3;
constexpr int kRightEncoderA = 0;
constexpr int kRightEncoderB = 1;

// Distance per Pulse
constexpr units::meter_t kWheelRadius = 3_in;
constexpr double kDpP =
    (2.0 * wpi::math::pi * kWheelRadius.to<double>()) / 2048.0;

}  // namespace Drivetrain

}  // namespace frc3512::Constants
