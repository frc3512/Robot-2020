// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <string_view>

#include <frc/RTNotifier.h>
#include <wpi/SmallVector.h>

#include "Constants.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

/**
 * A base class for subsystem controllers.
 *
 * The internal notifier will call ControllerPeriodic() on every controller
 * subsystem in the order they were constructed.
 */
class ControllerSubsystemBase : public SubsystemBase {
public:
    /**
     * Construct a controller subsystem.
     *
     * @param nodeName Name of pubsub node.
     */
    explicit ControllerSubsystemBase(std::string_view nodeName);

    virtual ~ControllerSubsystemBase();

    /**
     * Enable the controllers (start RTNotifier periodic).
     */
    static void Enable();

    /**
     * Disable the controllers (stop RTNotifier).
     */
    static void Disable();

    /**
     * Returns the time at which controllers were enabled.
     */
    static std::chrono::steady_clock::time_point GetStartTime();

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
    static frc::RTNotifier m_notifier;
    static std::chrono::steady_clock::time_point m_startTime;

    /**
     * Calls ControllerPeriodic() on each subsystem.
     */
    static void RunControllers();
};
}  // namespace frc3512
