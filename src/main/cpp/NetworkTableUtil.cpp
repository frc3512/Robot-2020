// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include "NetworkTableUtil.hpp"

namespace frc3512::NetworkTableUtil {

nt::NetworkTableEntry MakeEntry(const wpi::Twine& name,
                                std::initializer_list<double> defaultValue) {
    nt::NetworkTableInstance instance = nt::NetworkTableInstance::GetDefault();
    nt::NetworkTableEntry entry = instance.GetEntry(name);
    entry.SetDefaultDoubleArray(defaultValue);

    return entry;
}

}  // namespace frc3512::NetworkTableUtil
