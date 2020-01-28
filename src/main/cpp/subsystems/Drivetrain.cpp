// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Drivetrain.hpp"

#include <cmath>
#include <limits>
#include <string>

using namespace frc3512;
using namespace frc3512::Constants::Drivetrain;
using namespace frc3512::Constants::Robot;

Drivetrain::Drivetrain() : PublishNode("Drivetrain") {
    m_drive.SetDeadband(kJoystickDeadband);
    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);
    m_leftGrbx.SetInverted(true);
    m_drive.SetRightSideInverted(false);
}

void Drivetrain::Drive(double throttle, double turn, bool isQuickTurn) {
    m_drive.CurvatureDrive(throttle, turn, isQuickTurn);
}

void Drivetrain::SetLeftManual(double value) { m_leftGrbx.Set(value); }

void Drivetrain::SetRightManual(double value) { m_rightGrbx.Set(value); }

double Drivetrain::GetAngle() const { return m_gyro.GetAngle(); }

double Drivetrain::GetAngularRate() const { return m_gyro.GetRate(); }

void Drivetrain::ResetGyro() { m_gyro.Reset(); }

void Drivetrain::CalibrateGyro() { m_gyro.Calibrate(); }

double Drivetrain::GetLeftDisplacement() const { return m_leftEncoder.Get(); }

double Drivetrain::GetRightDisplacement() const { return m_rightEncoder.Get(); }

double Drivetrain::GetLeftRate() const { return m_leftEncoder.GetRate(); }

double Drivetrain::GetRightRate() const { return m_rightEncoder.GetRate(); }

void Drivetrain::ResetEncoders() {
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
}

void Drivetrain::Reset() {
    ResetEncoders();
    ResetGyro();
}

void Drivetrain::ProcessMessage(const HIDPacket& message) {
    if (GetRawButton(message, 0, 1)) {
        Drive(-message.y1 * 0.5, message.x2 * 0.5, GetRawButton(message, 1, 2));
    } else {
        Drive(-message.y1, message.x2, GetRawButton(message, 1, 2));
    }
}
