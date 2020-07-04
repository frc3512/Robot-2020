// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <string>

#include <wpi/StringRef.h>

namespace frc3512 {

struct ControllerLabel {
    ControllerLabel(wpi::StringRef name, wpi::StringRef unit)
        : name(name), unit(unit) {}

    std::string name;
    std::string unit;
};

}  // namespace frc3512
