// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <units/time.h>

namespace frc3512::sim {

/**
 * Steps simulation timing to when the next Notifier is scheduled.
 *
 * If there is no Notifier scheduled, simulation timing is left unchanged.
 *
 * @param delta An extra time delta that can be used for injecting scheduling
 *              jitter. This should be greater than zero so the notifier still
 *              triggers.
 */
void StepTimingToNextNotifier(units::second_t delta = 0_s);

}  // namespace frc3512::sim
