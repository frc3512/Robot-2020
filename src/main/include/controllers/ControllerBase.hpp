// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <array>

#include <Eigen/Core>
#include <units/time.h>
#include <wpi/StringRef.h>

#define NETWORK_LOGGING 0

#if NETWORK_LOGGING
#include "LiveGrapherControllerLogger.hpp"
#else
#include "CSVControllerLogger.hpp"
#endif

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
        : m_logger{controllerName, stateLabels, inputLabels, outputLabels} {}

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
     * Executes the control loop for a cycle.
     *
     * @param dt The time since the last call to this function.
     */
    virtual void UpdateController(units::second_t dt) = 0;

    /**
     * Executes the control loop for a cycle.
     *
     * @param dt          The time since the last call to Update().
     * @param elapsedTime The elapsed time to use for logging.
     */
    void Update(units::second_t dt, units::second_t elapsedTime) {
        m_logger.Log(elapsedTime, GetReferences(), GetStates(), GetInputs(),
                     GetOutputs());
        UpdateController(dt);
    }

private:
#if NETWORK_LOGGING
    LiveGrapherControllerLogger<States, Inputs, Outputs> m_logger;
#else
    CSVControllerLogger<States, Inputs, Outputs> m_logger;
#endif
};

}  // namespace frc3512
