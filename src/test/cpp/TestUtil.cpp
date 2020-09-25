// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "TestUtil.hpp"

#include <hal/HALBase.h>
#include <hal/simulation/MockHooks.h>
#include <hal/simulation/NotifierData.h>

namespace frc3512::sim {

void StepTimingToNextNotifier(units::second_t delta) {
    uint64_t nextTimeout = HALSIM_GetNextNotifierTimeout();
    if (nextTimeout == UINT64_MAX) {
        return;
    }

    int32_t status = 0;
    uint64_t curTime = HAL_GetFPGATime(&status);

    HALSIM_StepTiming(nextTimeout - curTime +
                      static_cast<uint64_t>(delta.to<double>() * 1e6));
}

}  // namespace frc3512::sim
