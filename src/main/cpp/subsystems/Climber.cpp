// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Climber.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Climber;

void Climber::SetTransverser(double speed) { m_transverser.Set(speed); }

void Climber::SetElevator(double speed) {
    m_elevatorLeft.Set(speed);
    m_elevatorRight.Set(speed);
}

void Climber::SpringElevator() {
    m_climbSole.Set(frc::DoubleSolenoid::Value::kForward);
}

void Climber::DisengageElevator() {
    m_climbSole.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Climber::ProcessMessage(const ButtonPacket& message) {
    if (message.topic == "Robot/AppendageStick" && message.button == 5 &&
        message.pressed) {
        SpringElevator();
    } else if (message.topic == "Robot/AppendageStick" && message.button == 3 &&
               message.pressed) {
        DisengageElevator();
    }
}

void Climber::ProcessMessage(const HIDPacket& message) {
    // TODO: Uncomment once climber and its motors are added to the robot
    // if (GetRawButton(message, 3, 2)) {
    //     SetTransverser(message.x4);
    // }
    // SetElevator(-message.y3);
}
