// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Notifier.h>
#include <wpi/SmallVector.h>

#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

/**
 * A base class for subsystems which use a feedback controller.
 *
 * The internal notifier will call ControllerPeriodic() on every controller
 * subsystem in the order they were constructed.
 */
class ControlledSubsystemBase : public SubsystemBase {
public:
    /**
     * Construct a controlled subsystem.
     */
    ControlledSubsystemBase();

    virtual ~ControlledSubsystemBase();

    /**
     * Enable the controllers (start Notifier periodic).
     */
    static void Enable();

    /**
     * Disable the controllers (stop Notifier).
     */
    static void Disable();

protected:
    /**
     * This function runs a controller asynchronously every 5 ms.
     *
     * Every controller is batched together, so they are synchronous with
     * respect to each other.
     */
    virtual void ControllerPeriodic() = 0;

private:
    static wpi::SmallVector<ControlledSubsystemBase*, 8> m_controllers;
    static frc::Notifier m_notifier;
};

}  // namespace frc3512
