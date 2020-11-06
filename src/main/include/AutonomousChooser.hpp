// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <functional>
#include <string>
#include <tuple>
#include <vector>

#include <frc/smartdashboard/Sendable.h>
#include <frc/smartdashboard/SendableBuilder.h>
#include <networktables/NetworkTableEntry.h>
#include <wpi/StringMap.h>
#include <wpi/StringRef.h>
#include <wpi/mutex.h>

namespace frc3512 {

/**
 * A convenience wrapper around a SendableChooser for managing, selecting, and
 * running autonomous modes.
 */
class AutonomousChooser : public frc::Sendable {
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

    ~AutonomousChooser();

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

    void InitSendable(frc::SendableBuilder& builder) override;

private:
    using AutonomousContainer =
        std::tuple<std::function<void()>, std::function<void()>>;

    wpi::mutex m_mutex;

    std::string m_defaultChoice;
    std::string m_selectedChoice;
    wpi::StringMap<AutonomousContainer> m_choices;
    std::vector<std::string> m_names;
    AutonomousContainer m_selectedAuton;

    nt::NetworkTableEntry m_defaultEntry;
    nt::NetworkTableEntry m_optionsEntry;
    nt::NetworkTableEntry m_selectedEntry;
    nt::NetworkTableEntry m_activeEntry;

    NT_EntryListener m_selectedListenerHandle;
};

}  // namespace frc3512
