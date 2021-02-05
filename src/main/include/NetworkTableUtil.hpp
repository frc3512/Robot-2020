// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <initializer_list>
#include <type_traits>

#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <wpi/Twine.h>

namespace frc3512::NetworkTableUtil {

/**
 * Creates a NetworkTable entry with a default value.
 *
 * @param name         Path of network table entry.
 * @param defaultValue The entry's initial value.
 */
template <typename T>
nt::NetworkTableEntry MakeEntry(const wpi::Twine& name, const T& defaultValue) {
    nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault();
    nt::NetworkTableEntry entry = instance.GetEntry(name);
    if constexpr (std::is_same_v<T, double>) {
        entry.SetDefaultDouble(defaultValue);
    } else if constexpr (std::is_same_v<T, bool>) {
        entry.SetDefaultBoolean(defaultValue);
    } else if constexpr (std::is_same_v<T, wpi::Twine>) {
        entry.SetDefaultString(defaultValue);
    }

    return entry;
}

/**
 * Creates a NetworkTable entry with a default value. This overload is for
 * double arrays.
 *
 * @param name         Path of network table entry.
 * @param defaultValue The entry's initial value.
 */
nt::NetworkTableEntry MakeEntry(const wpi::Twine& name,
                                std::initializer_list<double> defaultValue);

}  // namespace frc3512::NetworkTableUtil
