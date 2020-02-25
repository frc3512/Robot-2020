// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include <rev/ColorSensorV3.h>

// The color sensor library isn't built for macOS, so provide shims
#ifdef __llvm__
rev::ColorSensorV3::ColorSensorV3(frc::I2C::Port port) : m_i2c{port, 0x52} {}

frc::Color rev::ColorSensorV3::GetColor() { return frc::Color::kBlack; }
#endif
