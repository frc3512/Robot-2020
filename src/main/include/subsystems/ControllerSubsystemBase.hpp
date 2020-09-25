// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Notifier.h>
#include <units/time.h>
#include <wpi/SmallVector.h>

#include "Constants.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

/**
 * A base class for subsystems which contain controllers.
 *
 * The internal notifier will call ControllerPeriodic() on every controller
 * subsystem in the order they were constructed.
 */
class ControllerSubsystemBase : public SubsystemBase {
public:
    /**
     * Construct a controller subsystem.
     */
    ControllerSubsystemBase();

    virtual ~ControllerSubsystemBase();

    /**
     * Enable the controllers (start Notifier periodic).
     */
    static void Enable();

    /**
     * Disable the controllers (stop Notifier).
     */
    static void Disable();

    /**
     * Returns the time at which controllers were enabled.
     */
    static units::second_t GetStartTime();

protected:
    /**
     * This function runs a controller asynchronously every 5 ms.
     *
     * Every controller is batched together, so they are synchronous with
     * respect to each other.
     */
    virtual void ControllerPeriodic() = 0;

private:
    static wpi::SmallVector<ControllerSubsystemBase*, 16> m_controllers;
    static frc::Notifier m_notifier;
    static units::second_t m_startTime;

    /**
     * Calls ControllerPeriodic() on each subsystem.
     */
    static void RunControllers();
};
}  // namespace frc3512
