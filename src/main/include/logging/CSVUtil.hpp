// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <string_view>

#include <units/time.h>

namespace frc3512 {

/**
 * Deletes CSV files in current directory.
 *
 * This function is a no-op on the roboRIO because it doesn't have
 * std::filesystem support.
 */
void DeleteCSVs();

/**
 * Returns the timestamp when the robot program started.
 */
units::second_t GetStartTime();

}  // namespace frc3512
