// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Eigen/Core>

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
};
