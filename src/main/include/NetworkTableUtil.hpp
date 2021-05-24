// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <initializer_list>
#include <string_view>

#include <networktables/NetworkTableEntry.h>

namespace frc3512::NetworkTableUtil {

/**
 * Creates a NetworkTable entry with a default double value of 0.
 *
 * @param name         Path of network table entry.
 * @param defaultValue The entry's initial value.
 */
nt::NetworkTableEntry MakeDoubleEntry(std::string_view name,
                                      double defaultValue = 0.0);

/**
 * Creates a NetworkTable entry with a default bool value of false.
 *
 * @param name         Path of network table entry.
 * @param defaultValue The entry's initial value.
 */
nt::NetworkTableEntry MakeBoolEntry(std::string_view name,
                                    bool defaultValue = false);

/**
 * Creates a NetworkTable entry with a default string value of "".
 *
 * @param name         Path of network table entry.
 * @param defaultValue The entry's initial value.
 */
nt::NetworkTableEntry MakeStringEntry(std::string_view name,
                                      std::string_view defaultValue = "");

/**
 * Creates a NetworkTable entry with a default double array value of {}.
 *
 * @param name         Path of network table entry.
 * @param defaultValue The entry's initial value.
 */
nt::NetworkTableEntry MakeDoubleArrayEntry(
    std::string_view name, std::initializer_list<double> defaultValue = {});

}  // namespace frc3512::NetworkTableUtil
