// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include "RealTimeRobot.hpp"

#include <stdexcept>

#include <fmt/format.h>
#include <frc/Notifier.h>
#include <frc/Threads.h>

#include "RTUtils.hpp"
#include "UnitsFormat.hpp"

using namespace frc3512;

RealTimeRobot::RealTimeRobot(units::second_t robotPeriodicAllocation,
                             units::second_t controllerPeriod, int priority)
    : m_controllerPeriod{controllerPeriod},
      m_nextOffset{robotPeriodicAllocation} {
    if (!frc::Notifier::SetHALThreadPriority(true, 40)) {
        throw std::runtime_error(
            "Setting HAL Notifier RT priority to 40 failed\n");
    }

    if (!frc::SetCurrentThreadPriority(true, priority)) {
        throw std::runtime_error(fmt::format(
            "Setting RealTimeRobot RT priority to {} failed\n", priority));
    }

    // Give FRC_NetCommDaemon RT priority 35 so it's not preempted by robot code
    // during high CPU utilization.
    SetProcessRTPriority("/usr/local/frc/bin/FRC_NetCommDaemon", 35);
}

void RealTimeRobot::Schedule(std::function<void()> func,
                             units::second_t allocation) {
    if (m_nextOffset > m_controllerPeriod) {
        throw std::runtime_error(
            fmt::format("Function scheduled at offset {} with allocation {} "
                        "exceeded controller period of {}",
                        m_nextOffset, allocation, m_controllerPeriod));
    }

    AddPeriodic(func, m_controllerPeriod, m_nextOffset);
    m_nextOffset += allocation;
}
