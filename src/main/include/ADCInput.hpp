// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/AnalogInput.h>

namespace frc3512 {

/**
 * Converts AnalogInput into DigitalInput.
 */
class ADCInput {
public:
    /**
     * Constructs an ADCInput.
     *
     * @param channel The channel number on the roboRIO to represent. 0-3 are
     *                on-board 4-7 are on the MXP port.
     */
    explicit ADCInput(int channel);

    /**
     * Returns true if 3.3V has been reached.
     */
    bool Get();

private:
    frc::AnalogInput m_input;
    bool m_isSwitched = false;
};

}  // namespace frc3512
