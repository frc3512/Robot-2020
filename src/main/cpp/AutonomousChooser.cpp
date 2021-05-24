// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "AutonomousChooser.hpp"

#include <stdexcept>

#include <fmt/core.h>
#include <frc/DriverStation.h>
#include <frc/Threads.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NTSendableBuilder.h>

#include "RealTimePriorities.hpp"

namespace frc3512 {

AutonomousChooser::AutonomousChooser(std::string_view name,
                                     std::function<void()> func,
                                     units::second_t duration) {
    m_defaultChoice = name;
    m_choices.try_emplace(std::string{name}, func, duration);
    m_names.emplace_back(name);

    m_selectedChoice = name;
    m_selectedAuton = &m_choices.find(name)->second;

    frc::SmartDashboard::PutData("Autonomous modes", this);

    m_selectedListenerHandle = m_selectedEntry.AddListener(
        [=](const nt::EntryNotification& event) {
            if (!event.value->IsString()) {
                return;
            }

            {
                std::scoped_lock lock{m_selectionMutex};
                m_selectedChoice = event.value->GetString();
            }

            m_activeEntry.SetString(m_selectedChoice);
        },
        NT_NOTIFY_IMMEDIATE | NT_NOTIFY_NEW | NT_NOTIFY_UPDATE |
            NT_NOTIFY_LOCAL);
}

AutonomousChooser::~AutonomousChooser() {
    CancelAutonomous();
    m_selectedEntry.RemoveListener(m_selectedListenerHandle);
}

void AutonomousChooser::AddAutonomous(std::string_view name,
                                      std::function<void()> func,
                                      units::second_t duration) {
    m_choices.try_emplace(std::string{name}, func, duration);
    m_names.emplace_back(name);

    m_optionsEntry.SetStringArray(m_names);
}

void AutonomousChooser::SelectAutonomous(std::string_view name) {
    {
        std::scoped_lock lock{m_selectionMutex};
        m_selectedChoice = name;
        auto it = m_choices.find(m_selectedChoice);
        if (it != m_choices.end()) {
            m_selectedAuton = &it->second;
        }
    }
    m_selectedEntry.SetString(name);
}

units::second_t AutonomousChooser::SelectedAutonomousDuration() const {
    std::scoped_lock lock{m_selectionMutex};
    return m_selectedAuton->duration;
}

const std::vector<std::string>& AutonomousChooser::GetAutonomousNames() const {
    return m_names;
}

bool AutonomousChooser::Suspend() {
    // Resume main thread by yielding the shared mutex to it. The main thread
    // sets m_resumedAuton back to true when yielding.
    m_resumedAuton = false;
    m_cond.notify_one();
    m_cond.wait(m_autonLock, [&] { return m_resumedAuton; });

    // Return true if the auton thread should continue, or false if it should
    // exit. IsEnabled() isn't checked here so that if the robot loses
    // connection to the DriverStation and is temporarily disabled, the
    // autonomous mode will later resume without losing progress.
    return frc::DriverStation::IsAutonomous() && !m_autonShouldExit;
}

bool AutonomousChooser::Suspend(std::function<bool()> cond) {
    while (!cond()) {
        if (!Suspend()) {
            return false;
        }
    }

    return true;
}

bool AutonomousChooser::SuspendFor(units::second_t duration) {
    frc::Timer timer;
    timer.Start();
    return Suspend([=] { return timer.HasElapsed(duration); });
}

void AutonomousChooser::AwaitAutonomous() {
    // Only start a new auton thread if one isn't currently suspended
    if (IsSuspended()) {
        return;
    }

    {
        std::scoped_lock lock{m_selectionMutex};
        fmt::print("{} autonomous\n", m_selectedChoice);
        auto it = m_choices.find(m_selectedChoice);
        if (it != m_choices.end()) {
            m_selectedAuton = &it->second;
        }
    }

    m_resumedAuton = true;
    m_autonThread = std::thread{[=] {
        // Wait for main thread to yield for first time
        m_autonLock.lock();

        m_autonRunning = true;
        m_selectedAuton->func();
        m_autonRunning = false;

        // Resume main thread by yielding the shared mutex to it
        m_resumedAuton = false;
        m_cond.notify_one();
        m_autonLock.unlock();
    }};
    if (!frc::SetThreadPriority(m_autonThread, true, kPrioAutonomousThread)) {
        throw std::runtime_error(fmt::format(
            "Setting RT priority to {} failed\n", kPrioAutonomousThread));
    }

    // Yield to auton thread for first time by yielding the shared mutex to it.
    // The auton thread sets m_resumedAuton back to false when yielding.
    m_cond.wait(m_mainLock, [&] { return !m_resumedAuton; });
}

void AutonomousChooser::ResumeAutonomous() {
    if (m_autonRunning) {
        // If auton thread is running, resume it by yielding the shared mutex to
        // it. The auton thread sets m_resumedAuton back to false when yielding.
        m_resumedAuton = true;
        m_cond.notify_one();
        m_cond.wait(m_mainLock, [&] { return !m_resumedAuton; });
    }

    // If auton thread just finished (that is, main thread just changed the
    // m_autonRunning flag), join it so a future call to AwaitAutonomous() can
    // start a new thread. Otherwise, it will be a no-op to allow resumption
    // after disconnects that disable the robot.
    if (m_autonThread.joinable() && !m_autonRunning) {
        m_autonThread.join();
    }
}

void AutonomousChooser::CancelAutonomous() {
    if (m_autonThread.joinable()) {
        // Resume the auton thread so it can exit. If the autonomous mode checks
        // the return value of Suspend() correctly, it will exit as expected. If
        // the autonomous mode has finished, ResumeAutonomous() is a no-op
        // because otherwise, it would hang waiting for a Suspend() call that
        // will never occur.
        m_autonShouldExit = true;
        ResumeAutonomous();

        // Don't join the thread here because ResumeAutonomous() will do so
        // automatically
    }
}

bool AutonomousChooser::IsSuspended() const {
    return m_autonThread.joinable() && m_autonRunning;
}

void AutonomousChooser::InitSendable(nt::NTSendableBuilder& builder) {
    builder.SetSmartDashboardType("String Chooser");

    builder.GetEntry("default").SetString(m_defaultChoice);

    m_optionsEntry = builder.GetEntry("options");
    m_optionsEntry.SetStringArray(m_names);

    m_selectedEntry = builder.GetEntry("selected");
    m_selectedEntry.SetString(m_defaultChoice);

    m_activeEntry = builder.GetEntry("active");
    m_activeEntry.SetString(m_defaultChoice);
}

}  // namespace frc3512
