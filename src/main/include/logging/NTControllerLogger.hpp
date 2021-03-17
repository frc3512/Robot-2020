// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <array>

#include <Eigen/Core>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <wpi/StringRef.h>
#include <wpi/Twine.h>

#include "logging/ControllerLabel.hpp"

namespace frc3512 {

/**
 * Logs NT entries for state-space controllers.
 */
template <int States, int Inputs, int Outputs>
class NTControllerLogger {
public:
    /**
     * Constructs a NTControllerLogger.
     *
     * @param controllerName Name of the controller log file.
     * @param stateLabels    Labels for states each consisting of its name and
     *                       unit.
     * @param inputLabels    Labels for inputs each consisting of its name and
     *                       unit.
     * @param outputLabels   Labels for outputs each consisting of its name and
     *                       unit.
     */
    NTControllerLogger(
        wpi::StringRef controllerName,
        const std::array<ControllerLabel, States>& stateLabels,
        const std::array<ControllerLabel, Inputs>& inputLabels,
        const std::array<ControllerLabel, Outputs>& outputLabels) {
        wpi::Twine entryPrefix = "/Diagnostics/" + controllerName;

        nt::NetworkTableInstance instance =
            nt::NetworkTableInstance::GetDefault();
        for (int state = 0; state < States; ++state) {
            const auto& label = stateLabels[state];
            m_refEntries[state] =
                instance.GetEntry(entryPrefix + "/References/" + label.name);
            m_stateEntries[state] =
                instance.GetEntry(entryPrefix + "/States/" + label.name);
        }
        for (int input = 0; input < Inputs; ++input) {
            const auto& label = inputLabels[input];
            m_inputEntries[input] =
                instance.GetEntry(entryPrefix + "/Inputs/" + label.name);
        }
        for (int output = 0; output < Outputs; ++output) {
            const auto& label = outputLabels[output];
            m_outputEntries[output] =
                instance.GetEntry(entryPrefix + "/Outputs/" + label.name);
        }
    }

    NTControllerLogger(NTControllerLogger&&) = default;
    NTControllerLogger& operator=(NTControllerLogger&&) = default;

    /**
     * Logs the references, states, inputs, and outputs.
     *
     * @param r Reference vector.
     * @param x State vector.
     * @param u Input vector.
     * @param y Output vector.
     */
    void Log(const Eigen::Matrix<double, States, 1>& r,
             const Eigen::Matrix<double, States, 1>& x,
             const Eigen::Matrix<double, Inputs, 1>& u,
             const Eigen::Matrix<double, Outputs, 1>& y) {
        for (int state = 0; state < States; ++state) {
            m_refEntries[state].SetDouble(r(state));
            m_stateEntries[state].SetDouble(x(state));
        }
        for (int input = 0; input < Inputs; ++input) {
            m_inputEntries[input].SetDouble(u(input));
        }
        for (int output = 0; output < Outputs; ++output) {
            m_outputEntries[output].SetDouble(y(output));
        }
    }

private:
    std::array<nt::NetworkTableEntry, States> m_refEntries;
    std::array<nt::NetworkTableEntry, States> m_stateEntries;
    std::array<nt::NetworkTableEntry, Inputs> m_inputEntries;
    std::array<nt::NetworkTableEntry, Outputs> m_outputEntries;
};

}  // namespace frc3512
