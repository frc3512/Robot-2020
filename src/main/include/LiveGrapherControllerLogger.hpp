// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <array>
#include <string>
#include <tuple>
#include <utility>

#include <Eigen/Core>
#include <units/time.h>
#include <wpi/StringRef.h>
#include <wpi/Twine.h>

#include "ControllerLabel.hpp"
#include "livegrapher/LiveGrapher.hpp"

namespace frc3512 {

LiveGrapher& GetGrapher();

/**
 * Sends data from state-space controllers to LiveGrapher client.
 */
template <int States, int Inputs, int Outputs>
class LiveGrapherControllerLogger {
public:
    /**
     * Constructs a LiveGrapherControllerLogger.
     *
     * @param controllerName Name of the controller.
     * @param stateLabels    Labels for states each consisting of its name and
     *                       unit.
     * @param inputLabels    Labels for inputs each consisting of its name and
     *                       unit.
     * @param outputLabels   Labels for outputs each consisting of its name and
     *                       unit.
     */
    LiveGrapherControllerLogger(
        wpi::StringRef controllerName,
        const std::array<ControllerLabel, States>& stateLabels,
        const std::array<ControllerLabel, Inputs>& inputLabels,
        const std::array<ControllerLabel, Outputs>& outputLabels)
        : m_referenceLabels{MakeReferenceLabels(controllerName, stateLabels)},
          m_stateLabels{MakeStateEstimateLabels(controllerName, stateLabels)},
          m_inputLabels{MakeInputLabels(controllerName, inputLabels)},
          m_outputLabels{MakeOutputLabels(controllerName, outputLabels)} {}

    /**
     * Logs the references, states, inputs, and outputs.
     *
     * @param time Timestamp for data (unused).
     * @param r    Reference vector.
     * @param x    State vector.
     * @param u    Input vector.
     * @param y    Output vector.
     */
    void Log(units::second_t time, const Eigen::Matrix<double, States, 1>& r,
             const Eigen::Matrix<double, States, 1>& x,
             const Eigen::Matrix<double, Inputs, 1>& u,
             const Eigen::Matrix<double, Outputs, 1>& y) {
        LogVector<States>(*liveGrapher, m_referenceLabels, r);
        LogVector<States>(*liveGrapher, m_stateLabels, x);
        LogVector<Inputs>(*liveGrapher, m_inputLabels, u);
        LogVector<Outputs>(*liveGrapher, m_outputLabels, y);
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
        LogVector<States>(*liveGrapher, m_referenceLabels, r);
        LogVector<States>(*liveGrapher, m_stateLabels, x);
        LogVector<Inputs>(*liveGrapher, m_inputLabels, u);
        LogVector<Outputs>(*liveGrapher, m_outputLabels, y);
    }

private:
    LiveGrapher* liveGrapher = &GetGrapher();
    std::array<std::string, States> m_referenceLabels;
    std::array<std::string, States> m_stateLabels;
    std::array<std::string, Inputs> m_inputLabels;
    std::array<std::string, Outputs> m_outputLabels;

    /**
     * Returns tuple of reference labels from state names.
     */
    static auto MakeReferenceLabels(
        wpi::StringRef controllerName,
        const std::array<ControllerLabel, States>& stateLabels) {
        return std::array{std::apply(
            [=](auto... labels) {
                return std::array{(controllerName + " " + labels.name +
                                   " reference (" + labels.unit + ")")
                                      .str()...};
            },
            ArrayToTuple(stateLabels))};
    }

    /**
     * Returns tuple of state estimate labels from state names.
     */
    static auto MakeStateEstimateLabels(
        wpi::StringRef controllerName,
        const std::array<ControllerLabel, States>& stateLabels) {
        return std::array{std::apply(
            [=](auto... labels) {
                return std::array{(controllerName + " " + labels.name +
                                   " estimate (" + labels.unit + ")")
                                      .str()...};
            },
            ArrayToTuple(stateLabels))};
    }

    /**
     * Returns tuple of input labels from input names.
     */
    static auto MakeInputLabels(
        wpi::StringRef controllerName,
        const std::array<ControllerLabel, Inputs>& inputLabels) {
        return std::array{std::apply(
            [=](auto... labels) {
                return std::array{(controllerName + " " + labels.name +
                                   " input (" + labels.unit + ")")
                                      .str()...};
            },
            ArrayToTuple(inputLabels))};
    }

    /**
     * Returns tuple of output labels from output names.
     */
    static auto MakeOutputLabels(
        wpi::StringRef controllerName,
        const std::array<ControllerLabel, Outputs>& outputLabels) {
        return std::array{std::apply(
            [=](auto... labels) {
                return std::array{(controllerName + " " + labels.name +
                                   " measurement (" + labels.unit + ")")
                                      .str()...};
            },
            ArrayToTuple(outputLabels))};
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

    template <size_t... I, typename T>
    void LogVectorImpl(LiveGrapher& liveGrapher,
                       const std::array<std::string, sizeof...(I)>& labels,
                       const T& vec, std::index_sequence<I...>) {
        (liveGrapher.AddData(labels[I], vec(I)), ...);
    }

    template <size_t... I, typename T>
    void LogVectorWithTimeImpl(
        LiveGrapher& liveGrapher, units::second_t time,
        const std::array<std::string, sizeof...(I)>& labels, const T& vec,
        std::index_sequence<I...>) {
        (liveGrapher.AddData(time, labels[I], vec(I)), ...);
    }

    template <int Rows, typename Indices = std::make_index_sequence<Rows>>
    void LogVector(LiveGrapher& liveGrapher,
                   const std::array<std::string, Rows>& labels,
                   const Eigen::Matrix<double, Rows, 1>& vec) {
        LogVectorImpl(liveGrapher, labels, vec, Indices{});
    }

    template <int Rows, typename Indices = std::make_index_sequence<Rows>>
    void LogVectorWithTime(LiveGrapher& liveGrapher, units::second_t time,
                           const std::array<std::string, Rows>& labels,
                           const Eigen::Matrix<double, Rows, 1>& vec) {
        LogVectorWithTimeImpl(liveGrapher, time, labels, vec, Indices{});
    }
};

}  // namespace frc3512
