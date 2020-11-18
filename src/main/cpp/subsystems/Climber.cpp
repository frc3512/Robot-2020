// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include <cmath>

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
    SetCANSparkMaxBusUsage(m_traverser, Usage::kMinimal);
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
    SetTraverser(appendageStick2.GetX());

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
    static constexpr double kDeadband = 0.02;
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
