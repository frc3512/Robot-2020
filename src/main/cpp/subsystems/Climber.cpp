// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include <cmath>

#include <frc/Joystick.h>
#include <wpi/MathExtras.h>

using namespace frc3512;
using namespace frc3512::Constants::Climber;
using namespace frc3512::Constants::Robot;

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
}
