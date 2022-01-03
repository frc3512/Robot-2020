// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <initializer_list>

#include <networktables/NetworkTableEntry.h>
#include <wpi/Twine.h>

namespace frc3512::NetworkTableUtil {

/**
 * Creates a NetworkTable entry with a default double value of 0.
 *
 * @param name         Path of network table entry.
 * @param defaultValue The entry's initial value.
 */
nt::NetworkTableEntry MakeDoubleEntry(const wpi::Twine& name,
                                      double defaultValue = 0.0);

/**
 * Creates a NetworkTable entry with a default bool value of false.
 *
 * @param name         Path of network table entry.
 * @param defaultValue The entry's initial value.
 */
nt::NetworkTableEntry MakeBoolEntry(const wpi::Twine& name,
                                    bool defaultValue = false);

/**
 * Creates a NetworkTable entry with a default string value of "".
 *
 * @param name         Path of network table entry.
 * @param defaultValue The entry's initial value.
 */
nt::NetworkTableEntry MakeStringEntry(const wpi::Twine& name,
                                      const wpi::Twine& defaultValue = "");

/**
 * Creates a NetworkTable entry with a default double array value of {}.
 *
 * @param name         Path of network table entry.
 * @param defaultValue The entry's initial value.
 */
nt::NetworkTableEntry MakeDoubleArrayEntry(
    const wpi::Twine& name, std::initializer_list<double> defaultValue = {});

}  // namespace frc3512::NetworkTableUtil
