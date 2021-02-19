// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

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
     * @param name           Name of autonomous mode.
     * @param func           Autonomous mode function.
     * @param checkAutonEnds True if autonomous mode should end within 15
     *                       seconds in unit tests.
     */
    AutonomousChooser(wpi::StringRef name, std::function<void()> func,
                      bool checkAutonEnds = true);

    ~AutonomousChooser() override;

    /**
     * Adds an autonomous mode.
     *
     * @param name           Name of autonomous mode.
     * @param func           Autonomous mode function.
     * @param checkAutonEnds True if autonomous mode should end within 15
     *                       seconds in unit tests.
     */
    void AddAutonomous(wpi::StringRef name, std::function<void()> func,
                       bool checkAutonEnds = true);

    /**
     * Sets the selected autonomous mode for unit testing purposes.
     *
     * @param name Name of autonomous mode.
     */
    void SelectAutonomous(wpi::StringRef name);

    /**
     * Returns true if the selected autonomous mode should end within 15 seconds
     * in unit tests.
     */
    bool CheckSelectedAutonomousEnds() const;

    /**
     * Returns a list of selectable autonomous modes for unit testing purposes.
     */
    const std::vector<std::string>& GetAutonomousNames() const;

    /**
     * Suspend the autonomous mode, yield to the main robot thread, and wait for
     * the main thread to resume it.
     *
     * Returns true if the autonomous mode was allowed to continue by the main
     * thread or false if the autonomous mode should exit.
     *
     * This function should only be called by the autonomous mode. A call by the
     * main thread will block indefinitely.
     */
    bool Suspend();

    /**
     * Starts the selected autonomous mode.
     *
     * This function should only be called by the main thread. It will block
     * until the autonomous mode suspends itself. This ensures the main thread
     * and autonomous mode won't race for resources.
     */
    void AwaitAutonomous();

    /**
     * Resume the autonomous mode.
     *
     * This function should only be called by the main thread. It will block
     * until the autonomous mode suspends itself. This ensures the main thread
     * and autonomous mode won't race for resources.
     *
     * This should also be called when switching from autonomous to other modes
     * so the autonomous mode can exit. Once it exits, this function is a no-op
     * until AwaitAutonomous() is called again.
     */
    void ResumeAutonomous();

    /**
     * Stops the auton thread if it's running.
     */
    void CancelAutonomous();

    /**
     * Returns true if autonomous mode was started and is currently suspended.
     */
    bool IsSuspended() const;

    void InitSendable(frc::SendableBuilder& builder) override;

private:
    struct AutonomousMode {
        std::function<void()> func = [] {};
        bool checkAutonEnds = true;
    };

    std::thread m_autonThread;
    wpi::mutex m_autonMutex;

    // The main thread's lock object for the auton mutex
    std::unique_lock<wpi::mutex> m_mainLock{m_autonMutex};

    // The auton thread's lock object for the auton mutex. It starts unlocked so
    // the main thread can execute first.
    std::unique_lock<wpi::mutex> m_autonLock{m_autonMutex, std::defer_lock};

    wpi::condition_variable m_cond;
    bool m_resumedAuton = false;
    bool m_autonRunning = false;
    bool m_autonShouldExit = false;

    mutable wpi::mutex m_selectionMutex;
    std::string m_defaultChoice;
    std::string m_selectedChoice;
    wpi::StringMap<AutonomousMode> m_choices;
    std::vector<std::string> m_names;
    AutonomousMode* m_selectedAuton;

    nt::NetworkTableEntry m_defaultEntry;
    nt::NetworkTableEntry m_optionsEntry;
    nt::NetworkTableEntry m_selectedEntry;
    nt::NetworkTableEntry m_activeEntry;

    NT_EntryListener m_selectedListenerHandle;
};

}  // namespace frc3512
