// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "rev/CANSparkMaxLowLevel.hpp"

#if !defined(__FRC_ROBORIO__)
namespace rev {
CANSparkMaxLowLevel::CANSparkMaxLowLevel(int deviceID, MotorType motorType) {}
CANSparkMaxLowLevel::~CANSparkMaxLowLevel() {}
CANError CANSparkMaxLowLevel::SetPeriodicFramePeriod(PeriodicFrame frame,
                                                     int periodMs) {
    return CANError::kOk;
}
}  // namespace rev
#endif
