// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <array>
#include <tuple>

#include <Eigen/Core>
#include <frc/logging/CSVLogFile.h>
#include <wpi/StringRef.h>
#include <wpi/Twine.h>

#include "CSVLogFileUtils.hpp"
#include "ControllerLabel.hpp"

namespace frc3512 {

/**
 * Generates CSV files for state-space controllers.
 */
template <int States, int Inputs, int Outputs>
class CSVControllerLogger {
public:
    /**
     * Constructs a CSVControllerLogger.
     *
     * @param controllerName Name of the controller log file.
     * @param stateLabels    Labels for states each consisting of its name and
     *                       unit.
     * @param inputLabels    Labels for inputs each consisting of its name and
     *                       unit.
     * @param outputLabels   Labels for outputs each consisting of its name and
     *                       unit.
     */
    CSVControllerLogger(
        wpi::StringRef controllerName,
        const std::array<ControllerLabel, States>& stateLabels,
        const std::array<ControllerLabel, Inputs>& inputLabels,
        const std::array<ControllerLabel, Outputs>& outputLabels)
        : m_stateLogger{(controllerName + " states").str(),
                        std::tuple_cat(MakeReferenceLabels(stateLabels),
                                       MakeStateEstimateLabels(stateLabels))},
          m_inputLogger{(controllerName + " inputs").str(),
                        MakeInputLabels(inputLabels)},
          m_outputLogger{(controllerName + " outputs").str(),
                         MakeOutputLabels(outputLabels)} {}

    /**
     * Logs the references, states, inputs, and outputs.
     *
     * @param time Timestamp for data.
     * @param r    Reference vector.
     * @param x    State vector.
     * @param u    Input vector.
     * @param y    Output vector.
     */
    void Log(units::second_t time, const Eigen::Matrix<double, States, 1>& r,
             const Eigen::Matrix<double, States, 1>& x,
             const Eigen::Matrix<double, Inputs, 1>& u,
             const Eigen::Matrix<double, Outputs, 1>& y) {
        LogVector2WithTime(m_stateLogger, time, r, x);
        LogVectorWithTime(m_inputLogger, time, u);
        LogVectorWithTime(m_outputLogger, time, y);
    }

    /**
     * Logs the references, states, inputs, and outputs.
     *
     * @param r    Reference vector.
     * @param x    State vector.
     * @param u    Input vector.
     * @param y    Output vector.
     */
    void Log(const Eigen::Matrix<double, States, 1>& r,
             const Eigen::Matrix<double, States, 1>& x,
             const Eigen::Matrix<double, Inputs, 1>& u,
             const Eigen::Matrix<double, Outputs, 1>& y) {
        LogVector2(m_stateLogger, r, x);
        LogVector(m_inputLogger, u);
        LogVector(m_outputLogger, y);
    }

private:
    frc::CSVLogFile m_stateLogger;
    frc::CSVLogFile m_inputLogger;
    frc::CSVLogFile m_outputLogger;

    /**
     * Returns tuple of reference labels from state names.
     */
    static auto MakeReferenceLabels(
        const std::array<ControllerLabel, States>& stateLabels) {
        return std::apply(
            [](auto... labels) {
                return std::make_tuple(
                    (labels.name + " reference (" + labels.unit + ")")...);
            },
            ArrayToTuple(stateLabels));
    }

    /**
     * Returns tuple of state estimate labels from state names.
     */
    static auto MakeStateEstimateLabels(
        const std::array<ControllerLabel, States>& stateLabels) {
        return std::apply(
            [](auto... labels) {
                return std::make_tuple(
                    (labels.name + " estimate (" + labels.unit + ")")...);
            },
            ArrayToTuple(stateLabels));
    }

    /**
     * Returns tuple of input labels from input names.
     */
    static auto MakeInputLabels(
        const std::array<ControllerLabel, Inputs>& inputLabels) {
        return std::apply(
            [](auto... labels) {
                return std::make_tuple(
                    (labels.name + " input (" + labels.unit + ")")...);
            },
            ArrayToTuple(inputLabels));
    }

    /**
     * Returns tuple of output labels from output names.
     */
    static auto MakeOutputLabels(
        const std::array<ControllerLabel, Outputs>& outputLabels) {
        return std::apply(
            [](auto... labels) {
                return std::make_tuple(
                    (labels.name + " measurement (" + labels.unit + ")")...);
            },
            ArrayToTuple(outputLabels));
    }

    template <typename Array, size_t... I>
    static auto ArrayToTupleImpl(const Array& a, std::index_sequence<I...>) {
        return std::make_tuple(a[I]...);
    }

    template <typename T, size_t N,
              typename Indices = std::make_index_sequence<N>>
    static auto ArrayToTuple(const std::array<T, N>& a) {
        return ArrayToTupleImpl(a, Indices{});
    }
};

}  // namespace frc3512
