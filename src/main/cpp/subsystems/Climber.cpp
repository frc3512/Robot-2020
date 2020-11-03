// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include <cmath>

#include <frc/Joystick.h>
#include <wpi/MathExtras.h>
#include <wpi/math>

#include "CANSparkMaxUtil.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Climber;
using namespace frc3512::Constants::Robot;

Climber::Climber() {
    SetCANSparkMaxBusUsage(m_elevator, Usage::kPositionOnly);
    SetCANSparkMaxBusUsage(m_traverser, Usage::kMinimal);
}

void Climber::SetTraverser(double speed) { m_traverser.Set(speed); }

void Climber::SetElevator(double speed) {
    // Checks if the elevator's position is within the limits
    // Top of travel is 52.75 inches IRL
    if ((speed > 0.02 && GetElevatorPosition() <= 1.1129_m) ||
        (speed < -0.02 && GetElevatorPosition() >= 0_m)) {
        // Unlock climber if it's being commanded to move
        m_pancake.Set(true);
        m_elevator.Set(speed);
    } else {
        m_pancake.Set(false);
        m_elevator.Set(0.0);
    }
}

units::meter_t Climber::GetElevatorPosition() {
    constexpr double kG = 1.0 / 20.0;  // Gear ratio

    double rotations = m_elevatorEncoder.GetPosition();
    return units::meter_t{0.04381 * wpi::math::pi * kG * rotations /
                          (1.0 + 0.014983 * wpi::math::pi * kG * rotations)};
}

void Climber::RobotPeriodic() {
    m_elevatorEncoderEntry.SetDouble(GetElevatorPosition().to<double>());
}

void Climber::TeleopPeriodic() {
    static frc::Joystick appendageStick1{kAppendageStick1Port};
    static frc::Joystick appendageStick2{kAppendageStick2Port};

    // Climber traverser
    if (appendageStick2.GetRawButton(2)) {
        static constexpr double kMinInput = 0.5;

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
        double x = appendageStick2.GetX();
        SetTraverser((1.0 - kMinInput) * x + kMinInput * wpi::sgn(x));
    } else {
        SetTraverser(0.0);
    }

    // Climber elevator
    if (appendageStick1.GetRawButton(1)) {
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
        m_elevator.Set(speed);
    } else {
        m_pancake.Set(false);
        m_elevator.Set(0.0);
    }
}
