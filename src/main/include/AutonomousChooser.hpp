// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <functional>
#include <map>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <networktables/NTSendable.h>
#include <networktables/NetworkTableEntry.h>
#include <units/time.h>
#include <wpi/condition_variable.h>
#include <wpi/mutex.h>
#include <wpi/sendable/SendableHelper.h>

namespace nt {
class NTSendableBuilder;
}  // namespace nt

namespace frc3512 {

/**
 * A convenience wrapper around a SendableChooser for managing, selecting, and
 * running autonomous modes.
 */
class AutonomousChooser : public nt::NTSendable,
                          public wpi::SendableHelper<AutonomousChooser> {
public:
    /**
     * Constructs an AutonomousChooser.
     *
     * Adds an autonomous mode that's run by default if no other autonomous mode
     * is selected.
     *
     * @param name     Name of autonomous mode.
     * @param func     Autonomous mode function.
     * @param duration Duration of the autonomous mode to enforce in unit tests.
     */
    AutonomousChooser(std::string_view name, std::function<void()> func,
                      units::second_t duration = 15_s);

    ~AutonomousChooser() override;

    /**
     * Adds an autonomous mode.
     *
     * @param name     Name of autonomous mode.
     * @param func     Autonomous mode function.
     * @param duration Duration of the autonomous mode to enforce in unit tests.
     */
    void AddAutonomous(std::string_view name, std::function<void()> func,
                       units::second_t duration = 15_s);

    /**
     * Sets the selected autonomous mode for unit testing purposes.
     *
     * @param name Name of autonomous mode.
     */
    void SelectAutonomous(std::string_view name);

    /**
     * Returns the selected autonomous mode's expected duration.
     */
    units::second_t SelectedAutonomousDuration() const;

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
     * Suspend the autonomous mode, yield to the main robot thread, and wait for
     * the main thread to resume it when the given condition is true.
     *
     * If the condition is initially true, the autonomous mode continues without
     * suspending.
     *
     * Returns true if the autonomous mode was allowed to continue by the main
     * thread or false if the autonomous mode should exit.
     *
     * This function should only be called by the autonomous mode. A call by the
     * main thread will block indefinitely.
     *
     * @param cond Predicate which returns false if suspending should be
     *             continued.
     */
    bool Suspend(std::function<bool()> cond);

    /**
     * Suspend the autonomous mode, yield to the main robot thread, and wait for
     * the main thread to resume it when the given time has elapsed.
     *
     * Returns true if the autonomous mode was allowed to continue by the main
     * thread or false if the autonomous mode should exit.
     *
     * This function should only be called by the autonomous mode. A call by the
     * main thread will block indefinitely.
     *
     * @param duration Length of time for which to suspend.
     */
    bool SuspendFor(units::second_t duration);

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

    /**
     * Initializes this Sendable object.
     *
     * @param builder sendable builder
     */
    void InitSendable(nt::NTSendableBuilder& builder) override;

private:
    struct AutonomousMode {
        std::function<void()> func = [] {};
        units::second_t duration = 15_s;

        AutonomousMode() = default;
        AutonomousMode(std::function<void()> func, units::second_t duration)
            : func{func}, duration{duration} {}
        AutonomousMode(const AutonomousMode&) = default;
        AutonomousMode& operator=(const AutonomousMode&) = default;
        AutonomousMode(AutonomousMode&&) = default;
        AutonomousMode& operator=(AutonomousMode&&) = default;
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
    std::map<std::string, AutonomousMode, std::less<>> m_choices;
    std::vector<std::string> m_names;
    AutonomousMode* m_selectedAuton;

    nt::NetworkTableEntry m_defaultEntry;
    nt::NetworkTableEntry m_optionsEntry;
    nt::NetworkTableEntry m_selectedEntry;
    nt::NetworkTableEntry m_activeEntry;

    NT_EntryListener m_selectedListenerHandle;
};

}  // namespace frc3512
