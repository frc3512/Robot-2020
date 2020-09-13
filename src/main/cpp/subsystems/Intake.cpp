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

void Intake::RobotPeriodic() {
    if (m_flywheel.IsReady()) {
        // If ready to shoot, run the conveyor, funnel, and intake up.
        SetConveyor(0.85);
        SetFunnel(0.4);
        SetArmMotor(ArmMotorDirection::kIntake);
    } else if (IsUpperSensorBlocked()) {
        // Otherwise, if the upper sensor is blocked, only allow arm actuation.
        SetConveyor(0.0);
        SetFunnel(0.0);
    } else if (IsLowerSensorBlocked()) {
        // If the upper sensor isn't blocked and the lower sensor is, run the
        // conveyor and funnel up.
        SetConveyor(0.70);
        SetFunnel(0.4);
    } else {
        // If nothing is blocked, stop the conveyor.
        SetConveyor(0.0);
    }
}

void Intake::TeleopPeriodic() {
    static frc::Joystick appendageStick2{kAppendageStick2Port};

    // If the shooter isn't ready (the intake is preoccupied when the shooter is
    // ready), allow manually actuating the funnel and arm.
    if (!m_flywheel.IsReady()) {
        if (appendageStick2.GetRawButton(4)) {
            SetFunnel(0.4);
            SetArmMotor(ArmMotorDirection::kIntake);
        } else if (appendageStick2.GetRawButton(6)) {
            SetFunnel(-0.4);
            SetArmMotor(ArmMotorDirection::kOuttake);
        } else {
            SetFunnel(0.0);
            SetArmMotor(ArmMotorDirection::kIdle);
        }
    }

    if (appendageStick2.GetRawButtonPressed(3)) {
        if (IsDeployed()) {
            Stow();
        } else {
            Deploy();
        }
    }
}
