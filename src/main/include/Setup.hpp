// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

namespace frc3512 {

/**
 * Ensure RT threads don't consume more than 95% of the CPU.
 */
void SetRTRuntimeLimit();

/**
 * The crond service occasionally uses 50% CPU and there's no cronjobs to run.
 */
void StopCrond();

}  // namespace frc3512
