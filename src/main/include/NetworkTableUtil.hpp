// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <initializer_list>
#include <type_traits>

#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <wpi/Twine.h>

namespace frc3512::NetworkTableUtil {

/**
 * Creates a networktable entry that pushes a default or starting value to the
 * networktable. Works for most types.
 *
 * @param name Key; Path of entry in network table.
 * @param defaultValue Set the type and value its going to start with
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
 * Creates a networktable entry that pushes a default value to the
 * networktable. Uses double array.
 *
 * @param name Key; path of entry in networktable.
 * @param defaultValue Set the double array type and value it's going to start
 * with.
 */
nt::NetworkTableEntry MakeEntry(const wpi::Twine& name,
                                std::initializer_list<double> defaultValue);

}  // namespace frc3512::NetworkTableUtil
