// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <units/units.h>

namespace frc3512::Constants {

namespace Robot {
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
constexpr int kRightHallPort = 0;
constexpr int kLeftHallPort = 1;

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
constexpr int kLowerSensorPort = 5;
constexpr int kUpperSensorPort = 4;

// Solenoid Ports
constexpr int kArmForward = 0;
constexpr int kArmReverse = 1;
}  // namespace Intake

namespace Climber {
// Side Elevator Motors
constexpr int kElevatorPortLeft = 2;
constexpr int kElevatorPortRight = 3;

// Transverser Motor
constexpr int kTransverserPort = 13;

// Solenoid
constexpr int kSolenoidForward = 3;
constexpr int kSolenoidReverse = 4;

}  // namespace Climber

constexpr auto kDt = 0.00505_s;
constexpr int kControllerPrio = 50;
}  // namespace frc3512::Constants
