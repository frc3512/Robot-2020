// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#if defined(__FRC_ROBORIO__)
#include <rev/CANSparkMax.h>

#else
#include <frc/SpeedController.h>
#include <rev/CANError.h>
#include <rev/CANSparkMaxLowLevel.h>

namespace rev {

class CANSparkMax : public CANSparkMaxLowLevel {
public:
    CANSparkMax(int deviceID, rev::CANSparkMax::MotorType motorType);

    ~CANSparkMax() override = default;

    void Set(double speed) override;

    double Get() const override;

    void SetInverted(bool isInverted) override;

    bool GetInverted() const override;

    void Disable() override;

    void StopMotor() override;

    void PIDWrite(double output) override;

private:
    double m_speed = 0.0;
};

}  // namespace rev
#endif
