// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/Intake.hpp"

#include <frc/DriverStation.h>
#include <frc/Joystick.h>

#include "CANSparkMaxUtil.hpp"
#include "subsystems/Flywheel.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Intake;
using namespace frc3512::Constants::Robot;

Intake::Intake(Flywheel& flywheel) : m_flywheel(flywheel) {
    SetCANSparkMaxBusUsage(m_funnelMotorLeft, Usage::kMinimal);
    m_funnelMotorLeft.SetSmartCurrentLimit(80);
    SetCANSparkMaxBusUsage(m_funnelMotorRight, Usage::kMinimal);
    m_funnelMotorRight.SetSmartCurrentLimit(80);
    SetCANSparkMaxBusUsage(m_conveyorMotor, Usage::kMinimal);
    m_conveyorMotor.SetSmartCurrentLimit(80);
}

void Intake::Deploy() { m_arm.Set(frc::DoubleSolenoid::kForward); }

void Intake::Stow() { m_arm.Set(frc::DoubleSolenoid::kReverse); }

bool Intake::IsDeployed() const {
    return m_arm.Get() == frc::DoubleSolenoid::kForward;
}

void Intake::Start() {
    SetArmMotor(ArmMotorDirection::kIntake);
    SetFunnel(0.4);
}

void Intake::Stop() {
    SetArmMotor(ArmMotorDirection::kIdle);
    SetFunnel(0.0);
}

void Intake::FeedBalls() {
    m_armMotor.Set(-1.0);
    SetFunnel(-1.0);
    m_conveyorMotor.Set(-1.0);
}

void Intake::SetConveyor(double speed) { m_conveyorMotor.Set(speed); }

bool Intake::IsConveyorRunning() const { return m_conveyorMotor.Get() != 0.0; }

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
        if (frc::DriverStation::GetInstance().IsOperatorControlEnabled()) {
            if (appendageStick2.GetRawButton(4)) {
                SetArmMotor(ArmMotorDirection::kIntake);
            } else if (appendageStick2.GetRawButton(6)) {
                SetArmMotor(ArmMotorDirection::kOuttake);
            } else {
                SetArmMotor(ArmMotorDirection::kIdle);
            }
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
        if (frc::DriverStation::GetInstance().IsOperatorControlEnabled()) {
            if (appendageStick2.GetRawButton(4)) {
                SetFunnel(0.4);
            } else if (appendageStick2.GetRawButton(6)) {
                SetFunnel(-0.4);
            } else {
                SetFunnel(0.0);
            }
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

    m_upperSensorEntry.SetBoolean(IsUpperSensorBlocked());
    m_lowerSensorEntry.SetBoolean(IsLowerSensorBlocked());

    m_intakeLog.Log(frc2::Timer::GetFPGATimestamp() - m_startTime, m_arm.Get(),
                    m_armMotor.Get());
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

void Intake::SetArmMotor(ArmMotorDirection direction) {
    if (direction == ArmMotorDirection::kIntake) {
        m_armMotor.Set(-0.5);
    } else if (direction == ArmMotorDirection::kOuttake) {
        m_armMotor.Set(0.5);
    } else {
        m_armMotor.Set(0.0);
    }
}

void Intake::SetFunnel(double speed) {
    m_funnelMotorLeft.Set(speed);
    m_funnelMotorRight.Set(speed);
}
