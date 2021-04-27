// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include "RealTimeRobot.hpp"

#if !defined(_MSC_VER)
#include <unistd.h>
#endif  // !defined(_MSC_VER)

#include <cstdlib>
#include <stdexcept>

#include <fmt/format.h>
#include <frc/DriverStation.h>
#include <frc/Notifier.h>
#include <frc/Threads.h>
#include <frc/livewindow/LiveWindow.h>

#include "RTUtils.hpp"
#include "UnitsFormat.hpp"

using namespace frc3512;

RealTimeRobot::RealTimeRobot(units::second_t robotPeriodicAllocation,
                             units::second_t controllerPeriod, int priority)
    : m_controllerPeriod{controllerPeriod},
      m_nextOffset{robotPeriodicAllocation} {
    // These warnings generate console prints that cause scheduling jitter
    frc::DriverStation::GetInstance().SilenceJoystickConnectionWarning(true);

    // This telemetry regularly causes loop overruns
    frc::LiveWindow::GetInstance()->DisableAllTelemetry();

    // Log NT data every 20ms instead of every 100ms for higher resolution
    // dashboard plots
    SetNetworkTablesFlushEnabled(true);

    if constexpr (!IsSimulation()) {
        // crond occasionally uses 50% CPU and there's no cronjobs to run
#if !defined(_MSC_VER)
        setuid(0);
#endif  // !defined(_MSC_VER)
        int status = std::system("/etc/init.d/crond stop");
        if (status != 0) {
            throw std::runtime_error(
                fmt::format("Failed to stop crond ({})", status));
        }
    }

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
