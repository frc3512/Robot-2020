// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Intake.hpp"

#include <frc/Joystick.h>

#include "CANSparkMaxUtil.hpp"
#include "subsystems/Flywheel.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Intake;
using namespace frc3512::Constants::Robot;

Intake::Intake(Flywheel& flywheel) : m_flywheel(flywheel) {
    SetCANSparkMaxBusUsage(m_funnelMotorLeft, Usage::kMinimal);
    SetCANSparkMaxBusUsage(m_funnelMotorRight, Usage::kMinimal);
    SetCANSparkMaxBusUsage(m_conveyorMotor, Usage::kMinimal);
}

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
    // See intake-io-table.md for the input-output table the logic below was
    // based on.
    static frc::Joystick appendageStick2{kAppendageStick2Port};

    // Arm logic
    if (m_flywheel.IsOn()) {
        if (m_flywheel.IsReady()) {
            SetArmMotor(ArmMotorDirection::kIntake);
        } else {
            SetArmMotor(ArmMotorDirection::kIdle);
        }
    } else {
        // Manual control
        if (appendageStick2.GetRawButton(4)) {
            SetArmMotor(ArmMotorDirection::kIntake);
        } else if (appendageStick2.GetRawButton(6)) {
            SetArmMotor(ArmMotorDirection::kOuttake);
        } else {
            SetArmMotor(ArmMotorDirection::kIdle);
        }
    }

    // Funnel logic
    if (m_flywheel.IsOn() || IsLowerSensorBlocked()) {
        if (m_flywheel.IsReady() ||
            (!m_flywheel.IsOn() && !IsUpperSensorBlocked() &&
             IsLowerSensorBlocked())) {
            SetFunnel(0.4);
        } else {
            SetFunnel(0.0);
        }
    } else {
        // Manual control
        if (appendageStick2.GetRawButton(4)) {
            SetFunnel(0.4);
        } else if (appendageStick2.GetRawButton(6)) {
            SetFunnel(-0.4);
        } else {
            SetFunnel(0.0);
        }
    }

    // Conveyor logic
    if (m_flywheel.IsReady()) {
        SetConveyor(0.85);
    } else if (!m_flywheel.IsOn() && !IsUpperSensorBlocked() &&
               IsLowerSensorBlocked()) {
        SetConveyor(0.7);
    } else {
        SetConveyor(0.0);
    }
}

void Intake::TeleopPeriodic() {
    static frc::Joystick appendageStick2{kAppendageStick2Port};

    if (appendageStick2.GetRawButtonPressed(3)) {
        if (IsDeployed()) {
            Stow();
        } else {
            Deploy();
        }
    }
}
