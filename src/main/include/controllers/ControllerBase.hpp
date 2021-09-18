// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <Eigen/Core>

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
    ControllerBase() = default;

    /**
     * Move constructor.
     */
    ControllerBase(ControllerBase&&) = default;

    /**
     * Move assignment operator.
     */
    ControllerBase& operator=(ControllerBase&&) = default;

    virtual ~ControllerBase() = default;

    /**
     * Returns the current references.
     *
     * See the State class in the derived class for what each element
     * corresponds to.
     */
    const Eigen::Vector<double, States>& GetReferences() const { return m_r; }

    /**
     * Returns the control inputs.
     *
     * See the Input class in the derived class for what each element
     * corresponds to.
     */
    const Eigen::Vector<double, Inputs>& GetInputs() const { return m_u; }

    /**
     * Returns the next output of the controller.
     *
     * @param x The current state x.
     */
    virtual Eigen::Vector<double, Inputs> Calculate(
        const Eigen::Vector<double, States>& x) = 0;

    /**
     * Returns the next output of the controller.
     *
     * @param x The current state x.
     * @param r The next reference r.
     */
    Eigen::Vector<double, Inputs> Calculate(
        const Eigen::Vector<double, States>& x,
        const Eigen::Vector<double, States>& r) {
        m_nextR = r;
        Eigen::Vector<double, Inputs> u = Calculate(x);
        m_r = m_nextR;
        return u;
    }

protected:
    /**
     * Controller reference for current timestep.
     */
    Eigen::Vector<double, States> m_r = Eigen::Vector<double, States>::Zero();

    /**
     * Controller reference for next timestep.
     */
    Eigen::Vector<double, States> m_nextR =
        Eigen::Vector<double, States>::Zero();

    /**
     * Controller output.
     */
    Eigen::Vector<double, Inputs> m_u = Eigen::Vector<double, Inputs>::Zero();
};

}  // namespace frc3512
