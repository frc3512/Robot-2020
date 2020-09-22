// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "LoggingUtil.hpp"

#include <networktables/NetworkTableInstance.h>

namespace frc3512 {

nt::NetworkTableEntry GetNTEntry(wpi::StringRef tableName,
                                 wpi::StringRef entryName) {
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable(tableName);
    return table->GetEntry(entryName);
}

}  // namespace frc3512
