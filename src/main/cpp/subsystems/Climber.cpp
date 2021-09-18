// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include <cmath>

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/StateSpaceUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/MathExtras.h>
#include <wpi/numbers>

#include "CANSparkMaxUtil.hpp"
#include "HWConfig.hpp"
#include "subsystems/Turret.hpp"

using namespace frc3512;
using namespace frc3512::HWConfig::Climber;

Climber::Climber(Turret& turret) : m_turret{turret} {
    SetCANSparkMaxBusUsage(m_elevator, Usage::kPositionOnly);
    m_elevator.SetSmartCurrentLimit(40);
    SetCANSparkMaxBusUsage(m_traverser, Usage::kMinimal);
    m_traverser.SetSmartCurrentLimit(40);

    m_matcher.AddColorMatch(kRedTarget);
    m_matcher.AddColorMatch(kBlueTarget);
    m_matcher.AddColorMatch(kGreenTarget);
    m_matcher.AddColorMatch(kYellowTarget);
}

units::meter_t Climber::GetElevatorPosition() {
    constexpr double kG = 1.0 / 20.0;  // Gear ratio

    if constexpr (frc::RobotBase::IsSimulation()) {
        return units::meter_t{m_elevatorSim.GetOutput(0)};
    } else {
        double rotations = -m_elevatorEncoder.GetPosition();
        return units::meter_t{
            0.04381 * wpi::numbers::pi * kG * rotations /
            (1.0 + 0.014983 * wpi::numbers::pi * kG * rotations)};
    }
}

bool Climber::HasPassedTopLimit() {
    // Top of travel is 52.75 inches IRL
    return GetElevatorPosition() > 1.1129_m;
}

bool Climber::HasPassedBottomLimit() { return GetElevatorPosition() < 0_m; }

units::volt_t Climber::GetElevatorMotorOutput() const {
    return units::volt_t{-m_elevator.Get()};
}

void Climber::RobotPeriodic() {
    m_elevatorEncoderEntry.SetDouble(GetElevatorPosition().value());
    m_changedColorNumEntry.SetDouble(m_changedColorCount);

    m_currentColor =
        m_matcher.MatchClosestColor(m_colorSensor.GetColor(), m_confidence);

    if (m_currentColor == kRedTarget) {
        m_colorSensorOutputEntry.SetString("Red");
    } else if (m_currentColor == kBlueTarget) {
        m_colorSensorOutputEntry.SetString("Blue");
    } else if (m_currentColor == kYellowTarget) {
        m_colorSensorOutputEntry.SetString("Yellow");
    } else if (m_currentColor == kGreenTarget) {
        m_colorSensorOutputEntry.SetString("Green");
    } else {
        m_colorSensorOutputEntry.SetString("No Color");
    }

    if (m_state == ControlPanelState::kInit) {
        m_colorStateMachineEntry.SetString("Init");
    } else if (m_state == ControlPanelState::kRotateWheel) {
        m_colorStateMachineEntry.SetString("Rotate Wheel");
    } else if (m_state == ControlPanelState::kStopOnColor) {
        m_colorStateMachineEntry.SetString("Stop on Color");
    }

    if constexpr (frc::RobotBase::IsSimulation()) {
        m_elevatorSim.SetInput(Eigen::Vector<double, 1>{
            -m_elevator.Get() * frc::RobotController::GetInputVoltage()});

        m_elevatorSim.Update(20_ms);
    }
}

void Climber::TeleopPeriodic() {
    static frc::Joystick appendageStick1{HWConfig::kAppendageStick1Port};
    static frc::Joystick appendageStick2{HWConfig::kAppendageStick2Port};

    // Climber traverser
    if (m_state == ControlPanelState::kInit) {
        SetTraverser(appendageStick2.GetX());
    }

    bool readyToClimb =
        m_debouncer.Calculate(appendageStick1.GetRawButton(1) &&
                              std::abs(appendageStick1.GetY()) > 0.02);
    if (!m_prevReadyToClimb && readyToClimb) {
        // Move the turret out of the way of the climber and set new soft limit.
        // Also, disable auto-aim.
        m_turret.SetGoal(units::radian_t{wpi::numbers::pi / 2}, 0_rad_per_s);
        m_turret.SetControlMode(TurretController::ControlMode::kClosedLoop);
        m_turret.SetCWLimit(Turret::kCWLimitForClimbing);
    } else if (!readyToClimb && GetElevatorPosition() < 1.5_in) {
        // Let turret move full range again once climber is back down
        m_turret.SetCWLimit(TurretController::kCWLimit);
    }
    m_prevReadyToClimb = readyToClimb;

    // Make sure the turret is out of the way of the climber elevator before
    // moving it
    if (appendageStick1.GetRawButton(1) && !m_turret.HasPassedCWLimit()) {
        SetElevator(-appendageStick1.GetY());
    } else {
        SetElevator(0.0);
    }

    // Control panel
    if (appendageStick1.GetRawButtonPressed(6)) {
        m_colorSensorArm.Set(0.5);
    } else if (appendageStick1.GetRawButtonPressed(7)) {
        m_colorSensorArm.Set(0.0);
    }

    if (appendageStick1.GetRawButtonPressed(8)) {
        m_state = ControlPanelState::kRotateWheel;
        m_prevColor = m_currentColor;
        m_startColor = m_currentColor;
        m_colorSensorArm.Set(0.5);
    } else if (appendageStick1.GetRawButtonPressed(9)) {
        m_state = ControlPanelState::kStopOnColor;
        m_colorSensorArm.Set(0.5);
    }

    RunControlPanelSM();
}

void Climber::TestPeriodic() {
    static frc::Joystick appendageStick1{HWConfig::kAppendageStick1Port};

    // Positive voltage should move climber in the positive X direction
    double speed = -appendageStick1.GetY();

    // Ignore soft limits so the user can manually reset the elevator before
    // rebooting the robot
    if (std::abs(speed) > 0.02) {
        // Unlock climber if it's being commanded to move
        m_pancake.Set(true);
        m_elevator.Set(-speed);
    } else {
        m_pancake.Set(false);
        m_elevator.Set(0.0);
    }
}

void Climber::SetTraverser(double speed) {
    static constexpr double kDeadband = 0.1;
    static constexpr double kMinInput = 0.5;

    // Apply a deadband to the input to avoid chattering
    if (std::abs(speed) < kDeadband) {
        speed = 0.0;
    }

    // This equation rescales the following function
    //
    //   [-1 .. 0) -> [-1 .. 0) and 0 -> 0 and (0 .. 1] -> (0 .. 1]
    //
    // to
    //
    //   [-1 .. 0) -> [-1 .. -0.5) and 0 -> 0 and (0 .. 1] -> (0.5 .. 1]
    //
    // This provides a minimum input of 0.5 in either direction to overcome
    // friction while still linearly increasing to 1.
    m_traverser.Set((1.0 - kMinInput) * speed + kMinInput * wpi::sgn(speed));
}

void Climber::SetElevator(double speed) {
    if ((speed > 0.02 && !HasPassedTopLimit()) ||
        (speed < -0.02 && !HasPassedBottomLimit())) {
        // Unlock climber if it's being commanded to move
        m_pancake.Set(true);
        m_elevator.Set(-speed);
    } else {
        m_pancake.Set(false);
        m_elevator.Set(0.0);
    }
}

void Climber::RunControlPanelSM() {
    // If climbing, stop control panel mechanism if the process never completed
    if (m_state != ControlPanelState::kInit && GetElevatorPosition() > 1.5_in) {
        SetTraverser(0.0);
        m_state = ControlPanelState::kInit;
    }

    if (m_state == ControlPanelState::kInit) {
    } else if (m_state == ControlPanelState::kRotateWheel) {
        SetTraverser(1.0);
        // Check that the wheel has spun to the same color as the beginning
        // color. If previous color is the same as the start color, control
        // panel hasn't been moved yet.
        if (m_currentColor == m_startColor && m_prevColor != m_startColor) {
            m_changedColorCount++;
        }

        if (m_prevColor != m_currentColor) {
            m_prevColor = m_currentColor;
        }

        // 3 to 5 rotations of the control panel are required in the match.
        // Each color appears twice on the control panel, and the state machine
        // rotates the wheel 3.5 times. 2 * 3.5 = 7, so when a color is detected
        // 7 times, 3.5 rotations have occurred.
        if (m_changedColorCount >= 7) {
            SetTraverser(0.0);
            m_colorSensorArm.Set(0.0);
            m_changedColorCount = 0;
            m_state = ControlPanelState::kInit;
        }
    } else if (m_state == ControlPanelState::kStopOnColor) {
        SetTraverser(1.0);

        // Read game-specific data from the Field Management System (FMS)
        char desiredColor = frc::DriverStation::GetGameSpecificMessage()[0];

        // If there's no game-specific data, stop the state machine
        if (desiredColor != 'Y' && desiredColor != 'R' && desiredColor != 'B' &&
            desiredColor != 'G') {
            m_state = ControlPanelState::kInit;
        }

        // If the color sensor is 90° away from the desired color, stop moving
        if ((desiredColor == 'Y' && m_currentColor == kGreenTarget) ||
            (desiredColor == 'R' && m_currentColor == kBlueTarget) ||
            (desiredColor == 'B' && m_currentColor == kRedTarget) ||
            (desiredColor == 'G' && m_currentColor == kYellowTarget)) {
            SetTraverser(0.0);
            m_colorSensorArm.Set(0.0);
            m_state = ControlPanelState::kInit;
        }
    }
}
