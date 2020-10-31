// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "rev/CANSparkMax.hpp"

#if !defined(__FRC_ROBORIO__)
namespace rev {
CANSparkMax::CANSparkMax(int deviceID, rev::CANSparkMax::MotorType motorType)
    : CANSparkMaxLowLevel(deviceID, motorType) {}
void CANSparkMax::Set(double speed) { m_speed = speed; }
double CANSparkMax::Get() const { return m_speed; }
void CANSparkMax::SetInverted(bool isInverted) {}
bool CANSparkMax::GetInverted() const { return false; }
void CANSparkMax::Disable() { Set(0.0); }
void CANSparkMax::StopMotor() { Disable(); }
void CANSparkMax::PIDWrite(double output) { Set(output); }
}  // namespace rev
#endif
