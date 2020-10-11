// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <array>

#include <Eigen/Core>
#include <frc2/Timer.h>
#include <units/time.h>
#include <wpi/StringRef.h>

#include "CSVControllerLogger.hpp"
#include "LiveGrapherControllerLogger.hpp"

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
        : m_csvLogger{controllerName, stateLabels, inputLabels, outputLabels},
          m_liveGrapher{controllerName, stateLabels, inputLabels,
                        outputLabels} {}

    /**
     * Enables the control loop.
     */
    void Enable() {
        // m_lastTime is reset so that a large time delta isn't generated from
        // Update() not being called in a while.
        m_lastTime = frc2::Timer::GetFPGATimestamp();
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
     * Returns the current references.
     *
     * See the State class in the derived class for what each element correspond
     * to.
     */
    virtual const Eigen::Matrix<double, States, 1>& GetReferences() const = 0;

    /**
     * Returns the current state estimate.
     *
     * See the State class in the derived class for what each element correspond
     * to.
     */
    virtual const Eigen::Matrix<double, States, 1>& GetStates() const = 0;

    /**
     * Returns the control inputs.
     *
     * See the Input class in the derived class for what each element
     * corresponds to.
     */
    virtual const Eigen::Matrix<double, Inputs, 1>& GetInputs() const = 0;

    /**
     * Returns the currently set local outputs.
     *
     * See the Output class in the derived class for what each element
     * corresponds to.
     */
    virtual const Eigen::Matrix<double, Outputs, 1>& GetOutputs() const = 0;

    /**
     * Logs the current controller information, then executes the control loop
     * for a cycle.
     */
    void UpdateAndLog() {
        auto now = frc2::Timer::GetFPGATimestamp();
        m_csvLogger.Log(now - m_startTime, GetReferences(), GetStates(),
                        GetInputs(), GetOutputs());
        m_liveGrapher.Log(now - m_startTime, GetReferences(), GetStates(),
                          GetInputs(), GetOutputs());

        Update(now - m_lastTime);
        m_lastTime = now;
    }

protected:
    /**
     * Executes the control loop for a cycle.
     *
     * @param dt The time since the last call to this function.
     */
    virtual void Update(units::second_t dt) = 0;

private:
    CSVControllerLogger<States, Inputs, Outputs> m_csvLogger;
    LiveGrapherControllerLogger<States, Inputs, Outputs> m_liveGrapher;

    units::second_t m_lastTime = frc2::Timer::GetFPGATimestamp();
    units::second_t m_startTime = frc2::Timer::GetFPGATimestamp();
    bool m_isEnabled = false;
};

}  // namespace frc3512
