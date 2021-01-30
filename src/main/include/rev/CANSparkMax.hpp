// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#if defined(__FRC_ROBORIO__)
#include <rev/CANSparkMax.h>

#else
#include <frc/SpeedController.h>
#include <rev/CANError.h>

#include "rev/CANEncoder.hpp"
#include "rev/CANSparkMaxLowLevel.hpp"

namespace rev {

/**
 * REV CANSparkMax shim.
 */
class CANSparkMax : public CANSparkMaxLowLevel {
public:
    CANSparkMax(int deviceID, rev::CANSparkMax::MotorType motorType)
        : CANSparkMaxLowLevel{deviceID, motorType} {}

    ~CANSparkMax() override = default;

    void Set(double speed) override { m_speed = speed; }

    double Get() const override { return m_speed; }

    void SetInverted(bool isInverted) override {}

    bool GetInverted() const override { return false; }

    void Disable() override { Set(0.0); }

    void StopMotor() override { Disable(); }

    void PIDWrite(double output) override { Set(output); }

    CANEncoder GetEncoder(CANEncoder::EncoderType sensorType =
                              CANEncoder::EncoderType::kHallSensor,
                          int countsPerRev = 0) {
        return CANEncoder{*this, sensorType, countsPerRev};
    }

    CANError SetSmartCurrentLimit(unsigned int limit) { return CANError::kOk; }

    CANError SetSmartCurrentLimit(unsigned int smallLimit,
                                  unsigned int freeLimit,
                                  unsigned int limitRPM = 20000) {
        return CANError::kOk;
    }

private:
    double m_speed = 0.0;
};

}  // namespace rev
#endif
