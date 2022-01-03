// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

/**
 * The hardware configuration namespace is for joystick ports and roboRIO
 * channel assignments.
 *
 * Other hardware configuration constants that won't change often and belong to
 * only one subsystem (like gear ratios) should be defined in the subsystem
 * itself instead.
 */
namespace frc3512::HWConfig {

/// Drive joystick 1 port
constexpr int kDriveStick1Port = 0;

/// Drive joystick 2 port
constexpr int kDriveStick2Port = 1;

/// Appendage joystick 1 port
constexpr int kAppendageStick1Port = 2;

/// Appendage joystick 2 port
constexpr int kAppendageStick2Port = 3;

namespace Drivetrain {
/// Left motor leader CAN ID
constexpr int kLeftMotorLeaderID = 16;

/// Left motor follower CAN ID
constexpr int kLeftMotorFollowerID = 1;

/// Right motor leader CAN ID
constexpr int kRightMotorLeaderID = 14;

/// Right motor follower CAN ID
constexpr int kRightMotorFollowerID = 15;

// Encoder channels
constexpr int kLeftEncoderA = 0;
constexpr int kLeftEncoderB = 1;
constexpr int kRightEncoderA = 2;
constexpr int kRightEncoderB = 3;

/// Left ultrasonic sensor channel
constexpr int kLeftUltrasonicChannel = 3;

/// Right ultrasonic sensor channel
constexpr int kRightUltrasonicChannel = 2;
}  // namespace Drivetrain

namespace Flywheel {
/// Left motor CAN ID
constexpr int kLeftMotorID = 9;

/// Right motor CAN ID
constexpr int kRightMotorID = 10;

/// Encoder channel A
constexpr int kEncoderA = 6;

/// Encoder channel B
constexpr int kEncoderB = 7;
}  // namespace Flywheel

namespace Turret {
/// Turret motor CAN ID
constexpr int kMotorID = 8;

/// Counterclockwise hall sensor digital input channel
constexpr int kCCWHallChannel = 0;

/// Clockwise hall sensor digital input channel
constexpr int kCWHallChannel = 1;

// Encoder digital input channel
constexpr int kEncoderChannel = 8;
}  // namespace Turret

namespace Intake {
/// Arm motor CAN ID
constexpr int kArmMotorID = 6;

/// Conveyor motor CAN ID
constexpr int kConveyorMotorID = 7;

/// Funnel motor CAN ID
constexpr int kFunnelLeftMotorID = 4;
constexpr int kFunnelRightMotorID = 5;

/// Lower proximity sensor digital input channel
constexpr int kLowerSensorChannel = 4;

/// Upper proximity sensor digital input channel
constexpr int kUpperSensorChannel = 5;

/// Arm solenoid channel
constexpr int kArmChannel = 0;
}  // namespace Intake

namespace Climber {
/// Elevator motor CAN ID
constexpr int kElevatorMotorID = 3;

/// Traverser motor CAN ID
constexpr int kTraverserMotorID = 13;

/// Climber lock solenoid channel
constexpr int kClimberLockChannel = 3;

/// Climber color sensor arm servo channel
constexpr int kColorSensorArmServoChannel = 9;
}  // namespace Climber

}  // namespace frc3512::HWConfig
