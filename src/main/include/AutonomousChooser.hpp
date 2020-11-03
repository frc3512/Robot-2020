// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <functional>
#include <string>
#include <tuple>
#include <vector>

#include <frc/smartdashboard/SendableChooser.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <wpi/StringRef.h>

namespace frc3512 {

/**
 * A convenience wrapper around a SendableChooser for managing, selecting, and
 * running autonomous modes.
 */
class AutonomousChooser {
public:
    /**
     * Constructs an AutonomousChooser.
     *
     * Adds an autonomous mode that's run by default if no other autonomous mode
     * is selected.
     *
     * @param name         Name of autonomous mode.
     * @param initFunc     Init() function for autonomous mode that will run in
     *                     AutonomousInit().
     * @param periodicFunc Periodic() function for autonomous mode that will run
     *                     in AutonomousPeriodic().
     */
    AutonomousChooser(wpi::StringRef name, std::function<void()> initFunc,
                      std::function<void()> periodicFunc);

    /**
     * Adds an autonomous mode.
     *
     * @param name         Name of autonomous mode.
     * @param initFunc     Init() function for autonomous mode that will run in
     *                     AutonomousInit().
     * @param periodicFunc Periodic() function for autonomous mode that will run
     *                     in AutonomousPeriodic().
     */
    void AddAutonomous(wpi::StringRef name, std::function<void()> initFunc,
                       std::function<void()> periodicFunc);

    /**
     * Runs the Init() function of the selected autonomous mode.
     */
    void RunAutonomousInit();

    /**
     * Runs the Periodic() function of the selected autonomous mode.
     */
    void RunAutonomousPeriodic();

    /**
     * Sets the selected autonomous mode for unit testing purposes.
     *
     * @param name Name of autonomous mode.
     */
    void SelectAutonomous(wpi::StringRef name);

    /**
     * Returns a list of selectable autonomous modes for unit testing purposes.
     */
    const std::vector<std::string>& GetAutonomousNames() const;

private:
    using AutonomousContainer =
        std::tuple<std::function<void()>, std::function<void()>>;
    AutonomousContainer m_selected;
    frc::SendableChooser<AutonomousContainer> m_chooser;
    std::vector<std::string> m_names;

    nt::NetworkTableInstance m_inst = nt::NetworkTableInstance::GetDefault();
    nt::NetworkTableEntry m_selectionEntry =
        m_inst.GetEntry("/SmartDashboard/Autonomous modes");
};

}  // namespace frc3512
