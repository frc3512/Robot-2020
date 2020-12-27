// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <string_view>

namespace frc3512 {

/**
 * Deletes CSV files in current directory.
 *
 * This function is a no-op on the roboRIO because it doesn't have
 * std::filesystem support.
 */
void DeleteCSVs();

}  // namespace frc3512
