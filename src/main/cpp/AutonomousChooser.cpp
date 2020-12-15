// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "AutonomousChooser.hpp"

#include <algorithm>

#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>

namespace frc3512 {

AutonomousChooser::AutonomousChooser(wpi::StringRef name,
                                     std::function<void()> func) {
    m_defaultChoice = name;
    m_choices[name] = func;
    m_names.emplace_back(name);

    m_selectedChoice = name;
    m_selectedAuton = &m_choices[name];

    frc::SmartDashboard::PutData("Autonomous modes", this);

    m_selectedListenerHandle = m_selectedEntry.AddListener(
        [=](const nt::EntryNotification& event) {
            if (!event.value->IsString()) {
                return;
            }

            {
                std::scoped_lock lock{m_mutex};
                m_selectedChoice = event.value->GetString();
            }

            m_activeEntry.SetString(m_selectedChoice);
        },
        NT_NOTIFY_IMMEDIATE | NT_NOTIFY_NEW | NT_NOTIFY_UPDATE |
            NT_NOTIFY_LOCAL);
}

AutonomousChooser::~AutonomousChooser() {
    EndAutonomous();
    m_selectedEntry.RemoveListener(m_selectedListenerHandle);
}

void AutonomousChooser::AddAutonomous(wpi::StringRef name,
                                      std::function<void()> func) {
    m_choices[name] = func;
    m_names.emplace_back(name);

    // Unlike std::map, wpi::StringMap elements are not sorted
    std::sort(m_names.begin(), m_names.end());

    m_optionsEntry.SetStringArray(m_names);
}

void AutonomousChooser::SelectAutonomous(wpi::StringRef name) {
    {
        std::scoped_lock lock{m_mutex};
        m_selectedChoice = name;
    }
    m_selectedEntry.SetString(name);
}

const std::vector<std::string>& AutonomousChooser::GetAutonomousNames() const {
    return m_names;
}

void AutonomousChooser::Yield() {
    m_awaitingAuton = false;
    m_cond.notify_one();
    m_cond.wait(m_autonLock, [&] { return m_awaitingAuton; });
}

void AutonomousChooser::Return() {
    m_awaitingAuton = false;
    m_cond.notify_one();
}

void AutonomousChooser::AwaitStartAutonomous() {
    {
        std::scoped_lock lock{m_mutex};
        fmt::print("{} autonomous\n", m_selectedChoice);
        m_selectedAuton = &m_choices[m_selectedChoice];
    }

    m_awaitingAuton = true;
    m_autonThread = std::thread{[=] {
        m_autonLock.lock();
        m_autonRunning = true;
        (*m_selectedAuton)();
        m_autonRunning = false;
        Return();
        m_autonLock.unlock();
    }};
    m_cond.wait(m_mainLock, [&] { return !m_awaitingAuton; });
}

void AutonomousChooser::AwaitRunAutonomous() {
    if (m_autonRunning) {
        m_awaitingAuton = true;
        m_cond.notify_one();
        m_cond.wait(m_mainLock, [&] { return !m_awaitingAuton; });
    }
}

void AutonomousChooser::EndAutonomous() {
    if (m_autonRunning) {
        m_awaitingAuton = true;
        m_cond.notify_one();
        m_cond.wait(m_mainLock, [&] { return !m_awaitingAuton; });
    }
    if (m_autonThread.joinable()) {
        m_autonThread.join();
    }
}

void AutonomousChooser::InitSendable(frc::SendableBuilder& builder) {
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
