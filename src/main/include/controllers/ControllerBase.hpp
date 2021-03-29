// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

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
    ControllerBase(ControllerBase&&) = default;
    ControllerBase& operator=(ControllerBase&&) = default;

    virtual ~ControllerBase() = default;

    /**
     * Returns the current references.
     *
     * See the State class in the derived class for what each element
     * corresponds to.
     */
    const Eigen::Matrix<double, States, 1>& GetReferences() const {
        return m_r;
    }

    /**
     * Returns the control inputs.
     *
     * See the Input class in the derived class for what each element
     * corresponds to.
     */
    const Eigen::Matrix<double, Inputs, 1>& GetInputs() const { return m_u; }

    /**
     * Returns the next output of the controller.
     *
     * @param x The current state x.
     */
    virtual Eigen::Matrix<double, Inputs, 1> Calculate(
        const Eigen::Matrix<double, States, 1>& x) = 0;

    /**
     * Returns the next output of the controller.
     *
     * @param x The current state x.
     * @param r The next reference r.
     */
    Eigen::Matrix<double, Inputs, 1> Calculate(
        const Eigen::Matrix<double, States, 1>& x,
        const Eigen::Matrix<double, States, 1>& r) {
        m_nextR = r;
        Eigen::Matrix<double, Inputs, 1> u = Calculate(x);
        m_r = m_nextR;
        return u;
    }

protected:
    // Controller reference
    Eigen::Matrix<double, States, 1> m_r =
        Eigen::Matrix<double, States, 1>::Zero();
    Eigen::Matrix<double, States, 1> m_nextR =
        Eigen::Matrix<double, States, 1>::Zero();

    Eigen::Matrix<double, Inputs, 1> m_u =
        Eigen::Matrix<double, Inputs, 1>::Zero();
};

}  // namespace frc3512
