// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "CachedJoystick.hpp"

#include <frc/DriverStation.h>

using namespace frc3512;

CachedJoystick::CachedJoystick(int port) : m_port(port) { Update(); }

void CachedJoystick::Update() {
    auto& ds = frc::DriverStation::GetInstance();

    // Update button states
    m_buttons = ds.GetStickButtons(m_port);

    // Update edge events
    m_buttonsPressed = 0;
    m_buttonsReleased = 0;
    for (int i = 1; i <= 12; ++i) {
        if (ds.GetStickButtonPressed(m_port, i)) {
            m_buttonsPressed |= 1 << (i - 1);
        } else if (ds.GetStickButtonReleased(m_port, i)) {
            m_buttonsReleased |= 1 << (i - 1);
        }
    }

    // Update POVs
    for (size_t i = 0; i < m_povs.size(); ++i) {
        m_povs[i] = ds.GetStickPOV(m_port, i);
    }

    // Update axes
    m_x = ds.GetStickAxis(m_port, 0);
    m_y = ds.GetStickAxis(m_port, 1);
}

bool CachedJoystick::GetRawButton(int button) const {
    return m_buttons & (1 << (button - 1));
}

bool CachedJoystick::GetRawButtonPressed(int button) const {
    return m_buttonsPressed & (1 << (button - 1));
}

bool CachedJoystick::GetRawButtonReleased(int button) const {
    return m_buttonsReleased & (1 << (button - 1));
}

int CachedJoystick::GetPOV(int pov) const { return m_povs[pov]; }

double CachedJoystick::GetX(frc::GenericHID::JoystickHand hand) const {
    return m_x;
}

double CachedJoystick::GetY(frc::GenericHID::JoystickHand hand) const {
    return m_y;
}
