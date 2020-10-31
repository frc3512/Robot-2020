// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "rev/CANEncoder.hpp"

#if !defined(__FRC_ROBORIO__)
namespace rev {
CANEncoder::CANEncoder(CANSparkMax& device, EncoderType sensorType,
                       int counts_per_rev) {}
double CANEncoder::GetPosition() { return 0.0; }
}  // namespace rev
#endif
