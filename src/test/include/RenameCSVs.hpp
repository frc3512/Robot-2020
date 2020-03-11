// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <string_view>

/**
 * Rename CSVs to avoid them being overwritten by future test runs.
 *
 * @param prefix Prefix for turret CSV filename
 * @param name Start of CSV filename.
 */
void RenameCSVs(std::string_view prefix, std::string_view name);
