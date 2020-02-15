// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

#include <frc/DriverStation.h>
#include <wpi/raw_ostream.h>

#include "Robot.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Climber;
using namespace frc3512::Constants::Robot;

Climber::Climber() : PublishNode("Climber") {}

void Climber::SetTransverser(double speed) { m_transverser.Set(speed); }

void Climber::SetElevator(double speed) {
    m_armLeft.Set(speed);
    m_armRight.Set(speed);
}

void Climber::ForwardClimb() { m_climbSole.Set(frc::DoubleSolenoid::Value::kForward); };

void Climber::ReverseClimb() { m_climbSole.Set(frc::DoubleSolenoid::Value::kReverse); };

void Climber::LockClimb() { m_climbSole.Set(frc::DoubleSolenoid::Value::kOff); };

void Climber::ProcessMessage(const ButtonPacket& message) {
    if (message.topic == "Robot/AppendageStick" && message.button == 7 &&
        message.pressed) {
        
        m_winch.Set(1.0);
    } else if (message.topic == "Robot/AppendageStick" && message.button == 8 &&
               message.pressed) {
        m_winch.Set(-1.0);
    } else {
        m_winch.Set(0.0);
        LockClimb();
    }
}
