// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#if defined(__FRC_ROBORIO__)
#include <rev/CANSparkMaxLowLevel.h>

#else
#include <frc/SpeedController.h>
#include <rev/CANError.h>

namespace rev {

/**
 * REV CANSparkMaxLowLevel shim.
 */
class CANSparkMaxLowLevel : public frc::SpeedController {
public:
    enum class MotorType { kBrushed = 0, kBrushless = 1 };

    enum class PeriodicFrame { kStatus0 = 0, kStatus1 = 1, kStatus2 = 2 };

    CANSparkMaxLowLevel(int deviceID, MotorType motorType) {}

    virtual ~CANSparkMaxLowLevel() {}

    /**
     * Set periodic status frame period.
     */
    CANError SetPeriodicFramePeriod(PeriodicFrame frame, int periodMs) {
        return CANError::kOk;
    }
};

}  // namespace rev
#endif
