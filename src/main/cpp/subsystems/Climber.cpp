// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include <cmath>

#include <wpi/MathExtras.h>

using namespace frc3512;
using namespace frc3512::Constants::Climber;

void Climber::SetTransverser(double speed) { m_transverser.Set(speed); }

void Climber::SetElevator(double speed) { m_elevator.Set(speed); }

void Climber::ProcessMessage(const HIDPacket& message) {
    if (GetRawButton(message, 3, 2)) {
        SetTransverser(message.x4 * 0.5 + wpi::sgn(message.x4) * 0.5);
    } else {
        SetTransverser(0.0);
    }
    if (GetRawButton(message, 2, 1)) {
        SetElevator(std::abs(message.y3));
    } else {
        SetElevator(0.0);
    }
}
