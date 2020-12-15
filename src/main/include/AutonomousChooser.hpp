// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <atomic>
#include <functional>
#include <string>
#include <thread>
#include <vector>

#include <frc/smartdashboard/Sendable.h>
#include <frc/smartdashboard/SendableBuilder.h>
#include <networktables/NetworkTableEntry.h>
#include <wpi/StringMap.h>
#include <wpi/StringRef.h>
#include <wpi/condition_variable.h>
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
     * @param name Name of autonomous mode.
     * @param func Autonomous mode function.
     */
    AutonomousChooser(wpi::StringRef name, std::function<void()> func);

    ~AutonomousChooser();

    /**
     * Adds an autonomous mode.
     *
     * @param name Name of autonomous mode.
     * @param func Autonomous mode function.
     */
    void AddAutonomous(wpi::StringRef name, std::function<void()> func);

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

    /**
     * Yield to main robot thread and wait for next chance to run.
     *
     * This function should only be called by the autonomous mode. A call by the
     * main robot thread will block indefinitely.
     */
    void Yield();

    /**
     * Return to main robot thread.
     *
     * This function should only be called by the autonomous mode.
     */
    void Return();

    /**
     * Runs the selected autonomous mode function.
     */
    void AwaitStartAutonomous();

    /**
     * Notify autonomous mode to run.
     *
     * This function should only be called by the main robot thread. It will
     * block until the autonomous mode function waits to be run again. This
     * ensures the main robot thread and autonomous mode won't race for
     * resources.
     */
    void AwaitRunAutonomous();

    /**
     * Notify autonomous mode so it can exit.
     */
    void EndAutonomous();

    void InitSendable(frc::SendableBuilder& builder) override;

private:
    std::thread m_autonThread;
    wpi::mutex m_mutex;
    wpi::mutex m_autonMutex;
    std::unique_lock<wpi::mutex> m_mainLock{m_autonMutex};
    std::unique_lock<wpi::mutex> m_autonLock{m_autonMutex, std::defer_lock};
    wpi::condition_variable m_cond;
    bool m_awaitingAuton = false;
    bool m_autonRunning = false;

    std::string m_defaultChoice;
    std::string m_selectedChoice;
    wpi::StringMap<std::function<void()>> m_choices;
    std::vector<std::string> m_names;
    std::function<void()>* m_selectedAuton;

    nt::NetworkTableEntry m_defaultEntry;
    nt::NetworkTableEntry m_optionsEntry;
    nt::NetworkTableEntry m_selectedEntry;
    nt::NetworkTableEntry m_activeEntry;

    NT_EntryListener m_selectedListenerHandle;
};

}  // namespace frc3512
