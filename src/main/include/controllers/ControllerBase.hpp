// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/RTNotifier.h>
#include <units/units.h>

#include "Constants.hpp"

namespace frc3512 {

/**
 *  A base header file for various subsystem controllers.
 *
 *  Its purpose is to serve as a base class for controllers to use from,
 *  which is similar to how the SubsystemBase is used as a base for subsystems.
 */
class ControllerBase {
public:
    ControllerBase() = default;
    virtual ~ControllerBase() = default;

    /**
     * Enable the Controller (Start RTNotifier Periodic)
     */
    void Enable() { m_notifier.StartPeriodic(frc3512::Constants::kDt); }

    /**
     * Disable the Controller (Stop RTNotifier)
     */
    void Disable() { m_notifier.Stop(); }

    /**
     *  This function will be called asynchronously every 5 ms
     */
    virtual void Iterate() = 0;

private:
    frc::RTNotifier m_notifier{frc3512::Constants::kControllerPrio,
                               &ControllerBase::Iterate, this};
};
}  // namespace frc3512
