// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <array>
#include <atomic>
#include <stdexcept>
#include <thread>

#include <Eigen/Core>
#include <fmt/core.h>
#include <frc/RobotBase.h>
#include <frc/Threads.h>
#include <frc/logging/CSVLogFile.h>
#include <frc2/Timer.h>
#include <units/math.h>
#include <units/time.h>
#include <wpi/ConcurrentQueue.h>
#include <wpi/StringRef.h>
#include <wpi/Twine.h>

#include "RealTimeRobot.hpp"
#include "UnitsFormat.hpp"
#include "logging/CSVControllerLogger.hpp"
#include "logging/NTControllerLogger.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

/**
 * A base class for subsystems with controllers.
 *
 * State, Inputs, and Outputs indices should be specified what they represent in
 * the derived class.
 *
 * @tparam States the number of state estimates in the state vector
 * @tparam Inputs the number of control inputs in the input vector
 * @tparam Outputs the number of local outputs in the output vector
 */
template <int States, int Inputs, int Outputs>
class ControlledSubsystemBase : public SubsystemBase {
public:
    /**
     * Constructs a ControlledSubsystemBase.
     *
     * @param controllerName Name of the controller log file.
     * @param stateLabels    Labels for states each consisting of its name and
     *                       unit.
     * @param inputLabels    Labels for inputs each consisting of its name and
     *                       unit.
     * @param outputLabels   Labels for outputs each consisting of its name and
     *                       unit.
     */
    ControlledSubsystemBase(
        wpi::StringRef controllerName,
        const std::array<ControllerLabel, States>& stateLabels,
        const std::array<ControllerLabel, Inputs>& inputLabels,
        const std::array<ControllerLabel, Outputs>& outputLabels)
        : m_csvLogger{controllerName, stateLabels, inputLabels, outputLabels},
          m_ntLogger{controllerName, stateLabels, inputLabels, outputLabels},
          m_timingLogger{(controllerName + " timing").str(),
                         "Loop duration (ms)", "Scheduling period (ms)"} {
        m_entryThreadRunning = true;
        m_entryThread = std::thread{[=] { EntryThreadMain(); }};
    }

    ControlledSubsystemBase(ControlledSubsystemBase&&) = default;
    ControlledSubsystemBase& operator=(ControlledSubsystemBase&&) = default;

    ~ControlledSubsystemBase() override {
        m_entryThreadRunning = false;
        m_entryQueue.emplace();
        m_entryThread.join();
    }

    /**
     * Enables the control loop.
     */
    void Enable() {
        // m_lastTime is reset so that a large time delta isn't generated from
        // Update() not being called in a while.
        m_lastTime = frc2::Timer::GetFPGATimestamp() -
                     RealTimeRobot::kDefaultControllerPeriod;
        m_isEnabled = true;
    }

    /**
     * Disables the control loop.
     */
    void Disable() { m_isEnabled = false; }

    /**
     * Returns true if the control loop is enabled.
     */
    bool IsEnabled() const { return m_isEnabled; }

    /**
     * Returns the most recent timestep.
     */
    units::second_t GetDt() const { return m_dt; }

    /**
     * Runs periodic observer and controller update.
     */
    virtual void ControllerPeriodic() = 0;

    /**
     * Computes current timestep's dt.
     */
    void UpdateDt() {
        m_nowBegin = frc2::Timer::GetFPGATimestamp();
        m_dt = m_nowBegin - m_lastTime;

        if (m_dt == 0_s) {
            m_dt = RealTimeRobot::kDefaultControllerPeriod;
            fmt::print(stderr, "ERROR @ t = {}: dt = 0\n", m_nowBegin);
        }

        // Clamp spikes in scheduling latency
        if (m_dt > 10_ms) {
            m_dt = RealTimeRobot::kDefaultControllerPeriod;
        }
    }

    /**
     * Logs the current controller information.
     *
     * @param r The references for this timestep.
     * @param x The states for this timestep.
     * @param u The inputs for this timestep.
     * @param y The measurements for this timestep.
     */
    void Log(const Eigen::Matrix<double, States, 1>& r,
             const Eigen::Matrix<double, States, 1>& x,
             const Eigen::Matrix<double, Inputs, 1>& u,
             const Eigen::Matrix<double, Outputs, 1>& y) {
        m_entryQueue.emplace(m_nowBegin, frc2::Timer::GetFPGATimestamp(), r, x,
                             u, y);
        m_lastTime = m_nowBegin;
    }

private:
    struct LogEntry {
        bool valid = false;
        units::second_t nowBegin = 0_s;
        units::second_t nowEnd = 0_s;
        Eigen::Matrix<double, States, 1> r =
            Eigen::Matrix<double, States, 1>::Zero();
        Eigen::Matrix<double, States, 1> x =
            Eigen::Matrix<double, States, 1>::Zero();
        Eigen::Matrix<double, Inputs, 1> u =
            Eigen::Matrix<double, Inputs, 1>::Zero();
        Eigen::Matrix<double, Outputs, 1> y =
            Eigen::Matrix<double, Outputs, 1>::Zero();

        LogEntry() = default;

        LogEntry(units::second_t nowBegin, units::second_t nowEnd,
                 const Eigen::Matrix<double, States, 1>& r,
                 const Eigen::Matrix<double, States, 1>& x,
                 const Eigen::Matrix<double, Inputs, 1>& u,
                 const Eigen::Matrix<double, Outputs, 1>& y)
            : valid{true},
              nowBegin{nowBegin},
              nowEnd{nowEnd},
              r{r},
              x{x},
              u{u},
              y{y} {}
    };

    CSVControllerLogger<States, Inputs, Outputs> m_csvLogger;
    NTControllerLogger<States, Inputs, Outputs> m_ntLogger;
    frc::CSVLogFile m_timingLogger;

    units::second_t m_lastTime = frc::CSVLogFile::GetStartTime();
    units::second_t m_nowBegin = frc::CSVLogFile::GetStartTime();
    units::second_t m_dt = RealTimeRobot::kDefaultControllerPeriod;
    bool m_isEnabled = false;

    wpi::ConcurrentQueue<LogEntry> m_entryQueue;
    std::atomic<bool> m_entryThreadRunning{false};
    std::thread m_entryThread;

    void EntryThreadMain() {
        if (!frc::SetCurrentThreadPriority(
                true, RealTimeRobot::kDefaultPriority - 1)) {
            throw std::runtime_error(
                fmt::format("Setting logging thread RT priority to {} failed\n",
                            RealTimeRobot::kDefaultPriority - 1));
        }

        // Loops until the thread is told to exit and all remaining queue items
        // have been written to the logs
        while (m_entryThreadRunning || !m_entryQueue.empty()) {
            auto entry = m_entryQueue.pop();
            if (!entry.valid) {
                continue;
            }

            m_csvLogger.Log(entry.nowBegin - frc::CSVLogFile::GetStartTime(),
                            entry.r, entry.x, entry.u, entry.y);
            m_ntLogger.Log(entry.r, entry.x, entry.u, entry.y);

            m_timingLogger.Log(
                entry.nowBegin - frc::CSVLogFile::GetStartTime(),
                units::millisecond_t{entry.nowEnd - entry.nowBegin}
                    .to<double>(),
                units::millisecond_t{m_dt}.to<double>());
            m_lastTime = m_nowBegin;
        }
    }
};

}  // namespace frc3512
