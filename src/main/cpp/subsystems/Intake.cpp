// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Intake.hpp"

#include <frc/Joystick.h>

#include "subsystems/Flywheel.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Intake;
using namespace frc3512::Constants::Robot;

void Intake::Deploy() { m_arm.Set(frc::DoubleSolenoid::kForward); }

void Intake::Stow() { m_arm.Set(frc::DoubleSolenoid::kReverse); }

bool Intake::IsDeployed() const {
    return m_arm.Get() == frc::DoubleSolenoid::kForward;
}

void Intake::SetArmMotor(ArmMotorDirection armMotorState) {
    std::scoped_lock lock(m_armMotorMutex);
    if (armMotorState == ArmMotorDirection::kIntake) {
        m_armMotor.Set(-0.5);
    } else if (armMotorState == ArmMotorDirection::kOuttake) {
        m_armMotor.Set(0.5);
    } else {
        m_armMotor.Set(0.0);
    }
}

void Intake::SetFunnel(double speed) {
    if (!IsUpperSensorBlocked()) {
        m_funnelMotorLeft.Set(speed);
        m_funnelMotorRight.Set(speed);
    } else {
        m_funnelMotorLeft.Set(0.0);
        m_funnelMotorRight.Set(0.0);
    }
}

void Intake::FeedBalls() {
    m_armMotor.Set(-1.0);
    SetFunnel(-1.0);
    m_conveyorMotor.Set(-1.0);
}

void Intake::SetConveyor(double speed) { m_conveyorMotor.Set(speed); }

bool Intake::IsUpperSensorBlocked() const { return !m_upperSensor.Get(); }

bool Intake::IsLowerSensorBlocked() const { return !m_lowerSensor.Get(); }

void Intake::RobotPeriodic() {
    if (m_flywheel.GetGoal() > 0_rad_per_s && m_flywheel.AtGoal()) {
        SetConveyor(0.85);
    } else if (IsLowerSensorBlocked() && !IsUpperSensorBlocked()) {
        SetConveyor(0.70);
    } else {
        SetConveyor(0.0);
    }
}

void Intake::TeleopPeriodic() {
    static frc::Joystick appendageStick2{kAppendageStick2Port};

    if (appendageStick2.GetRawButton(4)) {
        SetArmMotor(ArmMotorDirection::kIntake);
        SetFunnel(0.4);
    } else if (appendageStick2.GetRawButton(6)) {
        SetArmMotor(ArmMotorDirection::kOuttake);
        SetFunnel(-0.4);
    } else {
        SetArmMotor(ArmMotorDirection::kIdle);
        SetFunnel(0.0);
    }

    if (appendageStick2.GetRawButtonPressed(3)) {
        if (IsDeployed()) {
            Stow();
        } else {
            Deploy();
        }
    }
}
