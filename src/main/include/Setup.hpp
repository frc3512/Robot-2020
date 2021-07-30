// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#pragma once

namespace frc3512 {

/**
 * The crond service occasionally uses 50% CPU and there's no cronjobs to run.
 */
void StopCrond();

}  // namespace frc3512
