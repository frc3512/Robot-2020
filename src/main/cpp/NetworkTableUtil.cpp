// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include "NetworkTableUtil.hpp"

#include <networktables/NetworkTableInstance.h>

namespace frc3512::NetworkTableUtil {

nt::NetworkTableEntry MakeEntry(const wpi::Twine& name, double defaultValue) {
    nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault();
    nt::NetworkTableEntry entry = instance.GetEntry(name);
    entry.SetDefaultDouble(defaultValue);

    return entry;
}

nt::NetworkTableEntry MakeEntry(const wpi::Twine& name, bool defaultValue) {
    nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault();
    nt::NetworkTableEntry entry = instance.GetEntry(name);
    entry.SetDefaultBoolean(defaultValue);

    return entry;
}

nt::NetworkTableEntry MakeEntry(const wpi::Twine& name,
                                const wpi::Twine& defaultValue) {
    nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault();
    nt::NetworkTableEntry entry = instance.GetEntry(name);
    entry.SetDefaultString(defaultValue);

    return entry;
}

nt::NetworkTableEntry MakeEntry(const wpi::Twine& name,
                                std::initializer_list<double> defaultValue) {
    nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault();
    nt::NetworkTableEntry entry = instance.GetEntry(name);
    entry.SetDefaultDoubleArray(defaultValue);

    return entry;
}

}  // namespace frc3512::NetworkTableUtil
