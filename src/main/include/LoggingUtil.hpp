// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <networktables/NetworkTableEntry.h>
#include <wpi/StringRef.h>

namespace frc3512 {

/**
 * Returns a NetworkTable entry from the given table in the default
 * NetworkTables instance.
 *
 * @param subtableName The table name.
 * @param entryName    The entry name.
 */
nt::NetworkTableEntry GetNTEntry(wpi::StringRef tableName,
                                 wpi::StringRef entryName);

}  // namespace frc3512
