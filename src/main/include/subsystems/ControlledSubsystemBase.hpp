// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <array>

#include <Eigen/Core>
#include <fmt/core.h>
#include <frc/RobotBase.h>
#include <frc/logging/CSVLogFile.h>
#include <frc2/Timer.h>
#include <units/math.h>
#include <units/time.h>
#include <wpi/StringRef.h>
#include <wpi/Twine.h>

#include "Constants.hpp"
#include "UnitsFormat.hpp"
#include "logging/CSVControllerLogger.hpp"
#include "logging/LiveGrapherControllerLogger.hpp"
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
#ifndef RUNNING_FRC_TESTS
        : m_csvLogger{controllerName, stateLabels, inputLabels, outputLabels},
          m_liveGrapher{controllerName, stateLabels, inputLabels, outputLabels},
          m_timingLogger{(controllerName + " timing").str(),
                         "Loop duration (ms)", "Scheduling period (ms)"} {
        // Write at least one data point to LiveGrapher for each dataset so they
        // are available when the robot is disabled.
        //
        // Graphs are only created when data has been logged for them,
        // ControllerPeriodic() is the only logger, and ControllerPeriodic()
        // only runs when the robot is enabled. Without the Log() call below,
        // the robot would need to be enabled at least once for any datasets to
        // be selectable.
        auto now = frc2::Timer::GetFPGATimestamp();
        Eigen::Matrix<double, States, 1> r;
        r.setZero();
        Eigen::Matrix<double, States, 1> x;
        x.setZero();
        Eigen::Matrix<double, Inputs, 1> u;
        u.setZero();
        Eigen::Matrix<double, Outputs, 1> y;
        y.setZero();
        m_liveGrapher.Log(now, r, x, u, y);
    }
#else
        : m_csvLogger{controllerName, stateLabels, inputLabels, outputLabels},
          m_timingLogger{(controllerName + " timing").str(),
                         "Loop duration (ms)", "Scheduling period (ms)"} {
    }
#endif

    ControlledSubsystemBase(ControlledSubsystemBase&&) = default;
    ControlledSubsystemBase& operator=(ControlledSubsystemBase&&) = default;

    ~ControlledSubsystemBase() override = default;

    /**
     * Enables the control loop.
     */
    void Enable() {
        // m_lastTime is reset so that a large time delta isn't generated from
        // Update() not being called in a while.
        m_lastTime = frc2::Timer::GetFPGATimestamp() - Constants::kDt;
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
            m_dt = Constants::kDt;
            fmt::print(stderr, "ERROR @ t = {}: dt = 0\n", m_nowBegin);
        }

        if (units::math::abs(m_dt - Constants::kDt) > 2.5_ms) {
            // Overruns are common in simulation, so don't print errors for them
            if constexpr (!frc::RobotBase::IsSimulation()) {
                fmt::print(stderr,
                           "ERROR @ t = {}: std::abs(dt - {}) > 2.5 ms where "
                           "dt = {}\n",
                           m_nowBegin, Constants::kDt, m_dt);
            }

            m_dt = Constants::kDt;
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
        m_csvLogger.Log(m_nowBegin - m_startTime, r, x, u, y);
#ifndef RUNNING_FRC_TESTS
        m_liveGrapher.Log(m_nowBegin - m_startTime, r, x, u, y);
#endif

        auto nowEnd = frc2::Timer::GetFPGATimestamp();
        m_timingLogger.Log(
            m_nowBegin - m_startTime,
            units::millisecond_t{nowEnd - m_nowBegin}.to<double>(),
            units::millisecond_t{m_dt}.to<double>());
        m_lastTime = m_nowBegin;
    }

private:
    CSVControllerLogger<States, Inputs, Outputs> m_csvLogger;
#ifndef RUNNING_FRC_TESTS
    LiveGrapherControllerLogger<States, Inputs, Outputs> m_liveGrapher;
#endif
    frc::CSVLogFile m_timingLogger;

    units::second_t m_lastTime = frc2::Timer::GetFPGATimestamp();
    units::second_t m_startTime = frc2::Timer::GetFPGATimestamp();
    units::second_t m_nowBegin = frc2::Timer::GetFPGATimestamp();
    units::second_t m_dt = Constants::kDt;
    bool m_isEnabled = false;
};

}  // namespace frc3512
