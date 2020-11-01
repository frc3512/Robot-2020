// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <string>

#include <wpi/StringRef.h>

namespace frc3512 {

/**
 * A container type for controller plot labels.
 *
 * Each label consists of a name and unit. Labels will be generated in the
 * format "name (unit)".
 */
struct ControllerLabel {
    /**
     * The label name.
     */
    std::string name;

    /**
     * The unit name.
     */
    std::string unit;

    /**
     * Constructs a ControllerLabel.
     *
     * @param name The label name.
     * @param unit The unit name.
     */
    ControllerLabel(wpi::StringRef name, wpi::StringRef unit)
        : name(name), unit(unit) {}
};

}  // namespace frc3512
