// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "AutonomousChooser.hpp"

#include <frc/smartdashboard/SmartDashboard.h>

namespace frc3512 {

AutonomousChooser::AutonomousChooser(wpi::StringRef name,
                                     std::function<void()> initFunc,
                                     std::function<void()> periodicFunc) {
    m_chooser.SetDefaultOption(name, {initFunc, periodicFunc});
    m_names.emplace_back(name);
    frc::SmartDashboard::PutData("Autonomous modes", &m_chooser);
    m_selected = m_chooser.GetSelected();
}

void AutonomousChooser::AddAutonomous(wpi::StringRef name,
                                      std::function<void()> initFunc,
                                      std::function<void()> periodicFunc) {
    m_chooser.AddOption(name, {initFunc, periodicFunc});
    m_names.emplace_back(name);
}

void AutonomousChooser::RunAutonomousInit() {
    m_selected = m_chooser.GetSelected();
    std::get<0>(m_selected)();
}

void AutonomousChooser::RunAutonomousPeriodic() { std::get<1>(m_selected)(); }

void AutonomousChooser::SelectAutonomous(wpi::StringRef name) {
    m_selectionEntry.SetString(name);
    m_inst.Flush();
    m_selected = m_chooser.GetSelected();
}

const std::vector<std::string>& AutonomousChooser::GetAutonomousNames() const {
    return m_names;
}

}  // namespace frc3512
