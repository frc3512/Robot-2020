// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include "ADCInput.hpp"

namespace frc3512 {

ADCInput::ADCInput(int channel) : m_input{channel} {}

bool ADCInput::Get() {
    double voltage = m_input.GetVoltage();
    if (voltage >= 3.3) {
        m_isSwitched = true;
    } else if (voltage <= 1.7) {
        m_isSwitched = false;
    }
    return m_isSwitched;
}

}  // namespace frc3512
