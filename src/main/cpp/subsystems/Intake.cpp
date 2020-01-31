// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Intake.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Intake;

Intake::Intake() : PublishNode("Intake") {}

void Intake::Deploy() { m_arm.Set(true); }

void Intake::Stow() { m_arm.Set(false); }

bool Intake::IsDeployed() const { return m_arm.Get(); }

void Intake::SetArmMotor(ArmMotorDirection armMotorState) {
    std::scoped_lock lock(m_armMotorMutex);
    if (armMotorState == ArmMotorDirection::kIntake) {
        m_armMotor.Set(1.0);
    } else if (armMotorState == ArmMotorDirection::kOuttake) {
        m_armMotor.Set(-1.0);
    } else {
        m_armMotor.Set(0.0);
    }
}

void Intake::SetFunnel(double speed) {
    m_funnelMotorLeft.Set(speed);
    m_funnelMotorRight.Set(speed);
}

void Intake::FeedBalls() {
    m_armMotor.Set(-1.0);
    SetFunnel(-1.0);
    m_conveyorMotor.Set(-1.0);
}

void Intake::SetConveyor(double speed) { m_conveyorMotor.Set(speed); }

bool Intake::IsUpperSensorBlocked() const { return !m_upperSensor.Get(); }

bool Intake::IsLowerSensorBlocked() const { return !m_lowerSensor.Get(); }

void Intake::ProcessMessage(const ButtonPacket& message) {
    if (message.topic == "Robot/AppendageStick2" && message.button == 4 &&
        message.pressed) {
        SetArmMotor(ArmMotorDirection::kIntake);
    } else if (message.topic == "Robot/AppendageStick2" &&
               message.button == 6 && message.pressed) {
        SetArmMotor(ArmMotorDirection::kOuttake);
    } else if (message.topic == "Robot/AppendageStick2" &&
               message.button == 4 && !message.pressed) {
        SetArmMotor(ArmMotorDirection::kIdle);
    } else if (message.topic == "Robot/AppendageStick2" &&
               message.button == 6 && !message.pressed) {
        SetArmMotor(ArmMotorDirection::kIdle);
    } else if (message.topic == "Robot/AppendageStick2" &&
               message.button == 3 && message.pressed) {
        if (IsDeployed()) {
            Stow();
        } else {
            Deploy();
        }
    }
}

void Intake::SubsystemPeriodic() {
    switch (m_state) {
        case State::kIdle: {
            // Wait until intake starts running
            if (m_armMotor.Get() > 0.25) {
                SetFunnel(1.0);
                SetConveyor(-1.0);
                m_conveyorTimer.Reset();
                m_conveyorTimer.Start();
                m_state = State::kDropBalls;
            }
            break;
        }
        case State::kDropBalls: {
            // Drop balls until lower proximity sensor is triggered or 2.5
            // seconds have passed to avoid gaps in balls
            if (IsLowerSensorBlocked() ||
                m_conveyorTimer.HasPeriodPassed(2.5)) {
                m_conveyorTimer.Stop();
                SetConveyor(1.0);
                m_state = State::kRaiseBalls;
            }
            break;
        }
        case State::kRaiseBalls: {
            // Move balls up conveyor until upper proximity sensor is triggered
            if (IsUpperSensorBlocked()) {
                SetConveyor(0.0);
                SetFunnel(0.0);
                m_state = State::kIdle;
            }
            break;
        }
    }
}
