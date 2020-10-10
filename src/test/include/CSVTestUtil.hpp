// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <string_view>

namespace frc3512 {
/**
 * Adds prefix to CSV filenames that haven't already been given a prefix.
 *
 * This avoids the files being overwritten by future test runs.
 *
 * @param prefix Prefix for CSV filename
 */
void AddPrefixToCSVs(std::string_view prefix);
}  // namespace frc3512
