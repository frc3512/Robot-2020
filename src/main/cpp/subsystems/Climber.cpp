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

    // Converts rotations to meters
    constexpr units::meter_t kDrumDiameter = 1_in;
    constexpr double kGearRatio = 1.0 / 20.0;
    m_elevatorEncoder.SetPositionConversionFactor(
        wpi::math::pi * kDrumDiameter.to<double>() * kGearRatio);
}

void Climber::SetTraverser(double speed) { m_traverser.Set(speed); }

void Climber::SetElevator(double speed) {
    if (std::abs(speed) > 0.02) {
        // Unlock climber if it's being commanded to move
        m_pancake.Set(true);
        m_elevator.Set(speed);
    } else {
        m_pancake.Set(false);
        m_elevator.Set(0.0);
    }
}

units::meter_t Climber::GetElevatorPosition() {
    return units::meter_t{m_elevatorEncoder.GetPosition()};
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
        SetElevator(appendageStick1.GetY());
    } else {
        SetElevator(0.0);
    }

    m_elevatorEncoderEntry.SetDouble(m_elevatorEncoder.GetPosition());
}
