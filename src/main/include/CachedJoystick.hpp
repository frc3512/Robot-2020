// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <stdint.h>

#include <array>

#include <frc/GenericHID.h>
#include <hal/DriverStationTypes.h>

namespace frc3512 {

/**
 * A joystick class that allows more control over when the button states are
 * updated and the edge triggers are cleared.
 *
 * Call Update() when the internal button state should be updated from the
 * Driver Station packet information.
 */
class CachedJoystick {
public:
    /**
     * Constructs a CachedJoystick.
     */
    explicit CachedJoystick(int port);

    /**
     * Clears the previous edge triggers, updates the internal button states,
     * and checks for new edge events.
     */
    void Update();

    /**
     * Get the button value (starting at button 1).
     *
     * The buttons are returned in a single 16 bit value with one bit
     * representing the state of each button. The appropriate button is returned
     * as a boolean value.
     *
     * This method returns true if the button is being held down at the time
     * that this method is being called.
     *
     * @param button The button number to be read (starting at 1)
     * @return The state of the button.
     */
    bool GetRawButton(int button) const;

    /**
     * Whether the button was pressed since the last check. Button indexes begin
     * at 1.
     *
     * This method returns true if the button went from not pressed to held down
     * since the last time Update() was called. This is useful if you only want
     * to call a function once when you press the button.
     *
     * @param button The button index, beginning at 1.
     * @return Whether the button was pressed since the last check.
     */
    bool GetRawButtonPressed(int button) const;

    /**
     * Whether the button was released since the last check. Button indexes
     * begin at 1.
     *
     * This method returns true if the button went from held down to not pressed
     * since the last time Update() was called. This is useful if you only want
     * to call a function once when you release the button.
     *
     * @param button The button index, beginning at 1.
     * @return Whether the button was released since the last check.
     */
    bool GetRawButtonReleased(int button) const;

    /**
     * Get the angle in degrees of a POV on the HID.
     *
     * The POV angles start at 0 in the up direction, and increase clockwise
     * (e.g. right is 90, upper-left is 315).
     *
     * @param pov The index of the POV to read (starting at 0)
     * @return the angle of the POV in degrees, or -1 if the POV is not pressed.
     */
    int GetPOV(int pov = 0) const;

    /**
     * Get the X value of the joystick.
     *
     * This depends on the mapping of the joystick connected to the current
     * port.
     *
     * @param hand This parameter is ignored for the Joystick class and is only
     *             here to complete the GenericHID interface.
     */
    double GetX(
        frc::GenericHID::JoystickHand hand = frc::GenericHID::kRightHand) const;

    /**
     * Get the Y value of the joystick.
     *
     * This depends on the mapping of the joystick connected to the current
     * port.
     *
     * @param hand This parameter is ignored for the Joystick class and is only
     *             here to complete the GenericHID interface.
     */
    double GetY(
        frc::GenericHID::JoystickHand hand = frc::GenericHID::kRightHand) const;

private:
    int m_port;
    uint32_t m_buttons = 0;
    uint32_t m_buttonsPressed = 0;
    uint32_t m_buttonsReleased = 0;
    std::array<int, HAL_kMaxJoystickPOVs> m_povs;

    double m_x = 0.0;
    double m_y = 0.0;
};

}  // namespace frc3512
