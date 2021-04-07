// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include <cmath>

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/StateSpaceUtil.h>
#include <wpi/MathExtras.h>
#include <wpi/math>

#include "CANSparkMaxUtil.hpp"
#include "Constants.hpp"
#include "subsystems/Turret.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Climber;
using namespace frc3512::Constants::Robot;

Climber::Climber(Turret& turret) : m_turret{turret} {
    SetCANSparkMaxBusUsage(m_elevator, Usage::kPositionOnly);
    m_elevator.SetSmartCurrentLimit(40);
    SetCANSparkMaxBusUsage(m_traverser, Usage::kMinimal);
    m_traverser.SetSmartCurrentLimit(40);
}

units::meter_t Climber::GetElevatorPosition() {
    constexpr double kG = 1.0 / 20.0;  // Gear ratio

    if constexpr (frc::RobotBase::IsSimulation()) {
        return units::meter_t{m_elevatorSim.GetOutput(0)};
    } else {
        double rotations = -m_elevatorEncoder.GetPosition();
        return units::meter_t{
            0.04381 * wpi::math::pi * kG * rotations /
            (1.0 + 0.014983 * wpi::math::pi * kG * rotations)};
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
    m_elevatorEncoderEntry.SetDouble(GetElevatorPosition().to<double>());

    if constexpr (frc::RobotBase::IsSimulation()) {
        m_elevatorSim.SetInput(frc::MakeMatrix<1, 1>(
            -m_elevator.Get() * frc::RobotController::GetInputVoltage()));

        m_elevatorSim.Update(20_ms);
    }
}

void Climber::TeleopPeriodic() {
    static frc::Joystick appendageStick1{kAppendageStick1Port};
    static frc::Joystick appendageStick2{kAppendageStick2Port};

    // Climber traverser
    if (m_state == ControlPanelState::kInit) {
        SetTraverser(appendageStick2.GetX());
    }

    if (appendageStick1.GetRawButton(1) &&
        std::abs(appendageStick1.GetY()) > 0.02) {
        // Move the turret out of the way of the climber and set new soft limit.
        // Also, disable auto-aim.
        m_turret.SetControlMode(TurretController::ControlMode::kClosedLoop);
        m_turret.SetGoal(TurretController::kCCWLimit, 0_rad_per_s);
        m_turret.SetCWLimit(Turret::kCWLimitForClimbing);
    } else if (GetElevatorPosition() < 1.5_in) {
        // Let turret move full range again once climber is back down
        m_turret.SetCWLimit(TurretController::kCWLimit);
    }

    // Make sure the turret is out of the way of the climber elevator before
    // moving it
    if (appendageStick1.GetRawButton(1) && !m_turret.HasPassedCWLimit()) {
        SetElevator(-appendageStick1.GetY());
    } else {
        SetElevator(0.0);
    }

    // Control panel
    if (appendageStick2.GetRawButtonPressed(7)) {
        m_controlPanelArm.Toggle();
    }

    RunControlPanelSM();
}

void Climber::TestPeriodic() {
    static frc::Joystick appendageStick1{kAppendageStick1Port};

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
    if (speed > 0.02 && !HasPassedTopLimit()) {
        // Unlock climber if it's being commanded to move
        m_pancake.Set(true);
        m_elevator.Set(-speed);
    } else if (speed < -0.02 && !HasPassedBottomLimit()) {
        // Unlock climber if it's being commanded to move
        m_pancake.Set(true);
        m_elevator.Set(-speed);
    } else {
        m_pancake.Set(false);
        m_elevator.Set(0.0);
    }
}

void Climber::RunControlPanelSM() {
    static constexpr frc::Color kRedTarget{0.561, 0.232, 0.114};
    static constexpr frc::Color kGreenTarget{0.197, 0.561, 0.240};
    static constexpr frc::Color kBlueTarget{0.143, 0.427, 0.429};
    static constexpr frc::Color kYellowTarget{0.361, 0.524, 0.113};

    static frc::Joystick appendageStick2{kAppendageStick2Port};
    static frc::Color prevColor;
    static int changedColorCount = 0;

    // If climbing, stop control panel mechanism if the process never completed
    if (m_state != ControlPanelState::kInit && GetElevatorPosition() > 1.5_in) {
        m_traverser.Set(0.0);
        m_state = ControlPanelState::kInit;
    }

    switch (m_state) {
        case ControlPanelState::kInit: {
            if (appendageStick2.GetRawButtonPressed(8)) {
                changedColorCount = 0;
                m_traverser.Set(1.0);
                m_state = ControlPanelState::kRotateWheel;
            } else if (appendageStick2.GetRawButtonPressed(9)) {
                prevColor = m_colorSensor.GetColor();
                m_traverser.Set(1.0);
                m_state = ControlPanelState::kStopOnColor;
            }
            break;
        }
        case ControlPanelState::kRotateWheel: {
            auto detectedColor = m_colorSensor.GetColor();

            if (prevColor != detectedColor) {
                prevColor = detectedColor;
                changedColorCount++;
            }

            // There are 8 color changes per control panel revolution
            if (changedColorCount == 32) {
                m_traverser.Set(0.0);
                m_state = ControlPanelState::kInit;
            }
            break;
        }
        case ControlPanelState::kStopOnColor: {
            auto gameData =
                frc::DriverStation::GetInstance().GetGameSpecificMessage();
            if (gameData[0] == 'Y') {
                if (m_colorSensor.GetColor() == kGreenTarget) {
                    m_traverser.Set(0.0);
                    m_state = ControlPanelState::kInit;
                }
            } else if (gameData[0] == 'R') {
                if (m_colorSensor.GetColor() == kBlueTarget) {
                    m_traverser.Set(0.0);
                    m_state = ControlPanelState::kInit;
                }
            } else if (gameData[0] == 'B') {
                if (m_colorSensor.GetColor() == kRedTarget) {
                    m_traverser.Set(0.0);
                    m_state = ControlPanelState::kInit;
                }
            } else if (gameData[0] == 'G') {
                if (m_colorSensor.GetColor() == kYellowTarget) {
                    m_traverser.Set(0.0);
                    m_state = ControlPanelState::kInit;
                }
            }
        }
    }
}
