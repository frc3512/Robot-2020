// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Intake.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Intake;

Intake::Intake() : PublishNode("Intake") {}

void Intake::Deploy() { m_arm.Set(frc::DoubleSolenoid::kForward); }

void Intake::Stow() { m_arm.Set(frc::DoubleSolenoid::kReverse); }

bool Intake::IsDeployed() const {
    return m_arm.Get() == frc::DoubleSolenoid::kForward;
}

void Intake::SetArmMotor(ArmMotorDirection armMotorState) {
    std::scoped_lock lock(m_armMotorMutex);
    if (armMotorState == ArmMotorDirection::kIntake) {
        m_armMotor.Set(0.5);
    } else if (armMotorState == ArmMotorDirection::kOuttake) {
        m_armMotor.Set(-0.5);
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
        message.pressed && !IsUpperSensorBlocked()) {
        SetArmMotor(ArmMotorDirection::kIntake);
        SetFunnel(0.4);
    } else if (message.topic == "Robot/AppendageStick2" &&
               message.button == 6 && message.pressed) {
        SetArmMotor(ArmMotorDirection::kOuttake);
        SetFunnel(-0.4);
    } else if (message.topic == "Robot/AppendageStick2" &&
               message.button == 4 && !message.pressed) {
        SetArmMotor(ArmMotorDirection::kIdle);
        SetFunnel(0.0);
    } else if (message.topic == "Robot/AppendageStick2" &&
               message.button == 6 && !message.pressed) {
        SetArmMotor(ArmMotorDirection::kIdle);
        SetFunnel(0.0);
    } else if (message.topic == "Robot/AppendageStick2" &&
               message.button == 3 && message.pressed) {
        if (IsDeployed()) {
            Stow();
        } else {
            Deploy();
        }
    }
}

void Intake::ProcessMessage(const CommandPacket& message) {
    if (message.topic == "Robot/TeleopInit" && !message.reply) {
        EnablePeriodic();
    } else if (message.topic == "Robot/AutonomousInit" && !message.reply) {
        EnablePeriodic();
    } else if (message.topic == "Robot/DisabledInit" && !message.reply) {
        DisablePeriodic();
    }
}

void Intake::SubsystemPeriodic() {
    if (IsLowerSensorBlocked() && !IsUpperSensorBlocked()) {
        SetConveyor(0.85);
    } else {
        SetConveyor(0.0);
    }
}
