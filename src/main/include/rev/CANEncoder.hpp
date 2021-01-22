// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#if defined(__FRC_ROBORIO__)
#include <rev/CANEncoder.h>

#else

namespace rev {

class CANSparkMax;

/**
 * REV CANEncoder shim.
 */
class CANEncoder {
public:
    enum class EncoderType {
        kNoSensor = 0,
        kHallSensor = 1,
        kQuadrature = 2,
        kSensorless = 3
    };

    explicit CANEncoder(
        CANSparkMax& device,
        EncoderType sensorType = CANEncoder::EncoderType::kHallSensor,
        int countsPerRev = 42) {}

    double GetPosition() { return 0.0; }
};

}  // namespace rev
#endif
