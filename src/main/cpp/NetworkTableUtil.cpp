// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "NetworkTableUtil.hpp"

#include <networktables/NetworkTableInstance.h>

namespace frc3512::NetworkTableUtil {

nt::NetworkTableEntry MakeDoubleEntry(std::string_view name,
                                      double defaultValue) {
    nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault();
    nt::NetworkTableEntry entry = instance.GetEntry(name);
    entry.SetDefaultDouble(defaultValue);

    return entry;
}

nt::NetworkTableEntry MakeBoolEntry(std::string_view name, bool defaultValue) {
    nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault();
    nt::NetworkTableEntry entry = instance.GetEntry(name);
    entry.SetDefaultBoolean(defaultValue);

    return entry;
}

nt::NetworkTableEntry MakeStringEntry(std::string_view name,
                                      std::string_view defaultValue) {
    nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault();
    nt::NetworkTableEntry entry = instance.GetEntry(name);
    entry.SetDefaultString(defaultValue);

    return entry;
}

nt::NetworkTableEntry MakeDoubleArrayEntry(
    std::string_view name, std::initializer_list<double> defaultValue) {
    nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault();
    nt::NetworkTableEntry entry = instance.GetEntry(name);
    entry.SetDefaultDoubleArray(defaultValue);

    return entry;
}

}  // namespace frc3512::NetworkTableUtil
