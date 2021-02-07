// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <initializer_list>

#include <networktables/NetworkTableEntry.h>
#include <wpi/Twine.h>

namespace frc3512::NetworkTableUtil {

/**
 * Creates a NetworkTable entry with a default value.
 *
 * @param name         Path of network table entry.
 * @param defaultValue The entry's initial value.
 */
template <typename T>
nt::NetworkTableEntry MakeEntry(const wpi::Twine& name, double defaultValue);

/**
 * Creates a NetworkTable entry with a default value.
 *
 * @param name         Path of network table entry.
 * @param defaultValue The entry's initial value.
 */
nt::NetworkTableEntry MakeEntry(const wpi::Twine& name, bool defaultValue);

/**
 * Creates a NetworkTable entry with a default value.
 *
 * @param name         Path of network table entry.
 * @param defaultValue The entry's initial value.
 */
nt::NetworkTableEntry MakeEntry(const wpi::Twine& name,
                                const wpi::Twine& defaultValue);

/**
 * Creates a NetworkTable entry with a default value.
 *
 * @param name         Path of network table entry.
 * @param defaultValue The entry's initial value.
 */
nt::NetworkTableEntry MakeEntry(const wpi::Twine& name,
                                std::initializer_list<double> defaultValue);

}  // namespace frc3512::NetworkTableUtil
