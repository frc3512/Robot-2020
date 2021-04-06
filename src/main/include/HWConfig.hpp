// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#pragma once

namespace frc3512::HWConfig {

// Joystick Ports
constexpr int kDriveStick1Port = 0;
constexpr int kDriveStick2Port = 1;
constexpr int kAppendageStick1Port = 2;
constexpr int kAppendageStick2Port = 3;

namespace Drivetrain {
// Motor Ports
constexpr int kLeftLeaderPort = 16;
constexpr int kLeftFollowerPort = 1;
constexpr int kRightLeaderPort = 14;
constexpr int kRightFollowerPort = 15;

// Encoder Ports
constexpr int kLeftEncoderA = 0;
constexpr int kLeftEncoderB = 1;
constexpr int kRightEncoderA = 2;
constexpr int kRightEncoderB = 3;

// Ultrasonic Port
constexpr int kLeftUltrasonicPort = 3;
constexpr int kRightUltrasonicPort = 2;
}  // namespace Drivetrain

namespace Flywheel {
constexpr int kLeftPort = 9;
constexpr int kRightPort = 10;
constexpr int kEncoderA = 6;
constexpr int kEncoderB = 7;
}  // namespace Flywheel

namespace Turret {
// Spark Max Port Values
constexpr int kPort = 8;

// Hall Sensor Ports
constexpr int kCCWHallPort = 0;
constexpr int kCWHallPort = 1;

// Encoder Values
constexpr int kEncoderPort = 8;
}  // namespace Turret

namespace Intake {
// Arm Motor Port
constexpr int kArmMotorPort = 6;

// Conveyor Motor Port
constexpr int kConveyorPort = 7;

// Funnel Motor Port
constexpr int kFunnelPortLeft = 4;
constexpr int kFunnelPortRight = 5;

// Proximity Sensor Ports (Digital Inputs)
constexpr int kLowerSensorPort = 4;
constexpr int kUpperSensorPort = 5;

// Solenoid Port
constexpr int kArmChannel = 0;
}  // namespace Intake

namespace Climber {
// Side Elevator Motors
constexpr int kElevatorPortRight = 3;

// Traverser Motor
constexpr int kTraverserPort = 13;

// Solenoid Port
constexpr int kClimberLock = 3;
}  // namespace Climber

}  // namespace frc3512::HWConfig
