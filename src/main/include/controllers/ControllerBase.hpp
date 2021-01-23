// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <array>

#include <Eigen/Core>
#include <fmt/core.h>
#include <frc/logging/CSVLogFile.h>
#include <frc2/Timer.h>
#include <units/time.h>
#include <wpi/StringRef.h>
#include <wpi/Twine.h>

#include "UnitsFormat.hpp"
#include "logging/CSVControllerLogger.hpp"
#include "logging/LiveGrapherControllerLogger.hpp"

namespace frc3512 {

/**
 * A base class for subsystem controllers.
 *
 * State, Inputs, and Outputs indices should be specified what they represent in
 * the derived class.
 *
 * @tparam States the number of state estimates in the state vector
 * @tparam Inputs the number of control inputs in the input vector
 * @tparam Outputs the number of local outputs in the output vector
 */
template <int States, int Inputs, int Outputs>
class ControllerBase {
public:
    /**
     * Constructs a ControllerBase.
     *
     * @param controllerName Name of the controller log file.
     * @param stateLabels    Labels for states each consisting of its name and
     *                       unit.
     * @param inputLabels    Labels for inputs each consisting of its name and
     *                       unit.
     * @param outputLabels   Labels for outputs each consisting of its name and
     *                       unit.
     */
    ControllerBase(wpi::StringRef controllerName,
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

    ControllerBase(ControllerBase&&) = default;
    ControllerBase& operator=(ControllerBase&&) = default;

    virtual ~ControllerBase() = default;

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
     * Set whether controller should run in closed-loop.
     */
    void SetClosedLoop(bool isClosedLoop) { m_isClosedLoop = isClosedLoop; }

    /**
     * Returns true if controller is running in closed-loop.
     */
    bool IsClosedLoop() const { return m_isClosedLoop; }

    /**
     * Returns the most recent timestep.
     */
    units::second_t GetDt() const { return m_dt; }

    /**
     * Returns the current references.
     *
     * See the State class in the derived class for what each element correspond
     * to.
     */
    virtual const Eigen::Matrix<double, States, 1>& GetReferences() const = 0;

    /**
     * Returns the current state estimate.
     *
     * See the State class in the derived class for what each element
     * corresponds to.
     */
    virtual const Eigen::Matrix<double, States, 1>& GetStates() const = 0;

    /**
     * Returns the control inputs.
     *
     * See the Input class in the derived class for what each element
     * corresponds to.
     */
    const Eigen::Matrix<double, Inputs, 1>& GetInputs() const { return m_u; }

    /**
     * Logs the current controller information, then executes the control loop
     * for a cycle.
     *
     * @param y The measurements for this timestep.
     */
    Eigen::Matrix<double, Inputs, 1> UpdateAndLog(
        const Eigen::Matrix<double, Outputs, 1>& y) {
        auto nowBegin = frc2::Timer::GetFPGATimestamp();
        m_dt = nowBegin - m_lastTime;

        m_csvLogger.Log(nowBegin - m_startTime, GetReferences(), GetStates(),
                        m_u, y);
#ifndef RUNNING_FRC_TESTS
        m_liveGrapher.Log(nowBegin - m_startTime, GetReferences(), GetStates(),
                          m_u, y);
#endif

        if (m_dt > 0_s) {
            m_u = Update(y, m_dt);
            auto nowEnd = frc2::Timer::GetFPGATimestamp();
            m_timingLogger.Log(
                nowBegin - m_startTime,
                units::millisecond_t{nowEnd - nowBegin}.to<double>(),
                units::millisecond_t{m_dt}.to<double>());
            m_lastTime = nowBegin;
        } else {
            fmt::print(stderr, "ERROR: dt = 0 @ t = {}\n", nowBegin);
        }
        return m_u;
    }

protected:
    /**
     * Executes the control loop for a cycle and returns the input.
     *
     * @param y  The measurements for this timestep.
     * @param dt The time since the last call to this function.
     */
    virtual Eigen::Matrix<double, Inputs, 1> Update(
        const Eigen::Matrix<double, Outputs, 1>& y, units::second_t dt) = 0;

private:
    CSVControllerLogger<States, Inputs, Outputs> m_csvLogger;
#ifndef RUNNING_FRC_TESTS
    LiveGrapherControllerLogger<States, Inputs, Outputs> m_liveGrapher;
#endif
    frc::CSVLogFile m_timingLogger;

    Eigen::Matrix<double, Inputs, 1> m_u =
        Eigen::Matrix<double, Inputs, 1>::Zero();
    units::second_t m_lastTime = frc2::Timer::GetFPGATimestamp();
    units::second_t m_startTime = frc2::Timer::GetFPGATimestamp();
    units::second_t m_dt = Constants::kDt;
    bool m_isEnabled = false;
    bool m_isClosedLoop = true;
};

}  // namespace frc3512
