// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <array>
#include <string_view>
#include <tuple>
#include <utility>

#include <Eigen/Core>
#include <fmt/format.h>
#include <frc/logging/CSVLogFile.h>
#include <units/time.h>

#include "logging/ControllerLabel.hpp"

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
        std::string_view controllerName,
        const std::array<ControllerLabel, States>& stateLabels,
        const std::array<ControllerLabel, Inputs>& inputLabels,
        const std::array<ControllerLabel, Outputs>& outputLabels)
        : m_stateLogger{fmt::format("{} states", controllerName),
                        std::tuple_cat(MakeReferenceLabels(stateLabels),
                                       MakeStateEstimateLabels(stateLabels))},
          m_inputLogger{fmt::format("{} inputs", controllerName),
                        MakeInputLabels(inputLabels)},
          m_outputLogger{fmt::format("{} outputs", controllerName),
                         MakeOutputLabels(outputLabels)} {}

    /**
     * Move constructor.
     */
    CSVControllerLogger(CSVControllerLogger&&) = default;

    /**
     * Move assignment operator.
     */
    CSVControllerLogger& operator=(CSVControllerLogger&&) = default;

    /**
     * Logs the references, states, inputs, and outputs.
     *
     * @param time Timestamp for data.
     * @param r    Reference vector.
     * @param x    State vector.
     * @param u    Input vector.
     * @param y    Output vector.
     */
    void Log(units::second_t time, const Eigen::Vector<double, States>& r,
             const Eigen::Vector<double, States>& x,
             const Eigen::Vector<double, Inputs>& u,
             const Eigen::Vector<double, Outputs>& y) {
        LogVector2WithTime<States, States>(m_stateLogger, time, r, x);
        LogVectorWithTime<Inputs>(m_inputLogger, time, u);
        LogVectorWithTime<Outputs>(m_outputLogger, time, y);
    }

    /**
     * Logs the references, states, inputs, and outputs.
     *
     * @param r    Reference vector.
     * @param x    State vector.
     * @param u    Input vector.
     * @param y    Output vector.
     */
    void Log(const Eigen::Vector<double, States>& r,
             const Eigen::Vector<double, States>& x,
             const Eigen::Vector<double, Inputs>& u,
             const Eigen::Vector<double, Outputs>& y) {
        LogVector2<States, States>(m_stateLogger, r, x);
        LogVector<Inputs>(m_inputLogger, u);
        LogVector<Outputs>(m_outputLogger, y);
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

    template <typename T, size_t... I>
    void LogVectorImpl(frc::CSVLogFile& logger, const T& vec,
                       std::index_sequence<I...>) {
        return logger.Log(vec(I)...);
    }

    template <typename T, size_t... I>
    void LogVectorWithTimeImpl(frc::CSVLogFile& logger, units::second_t time,
                               const T& vec, std::index_sequence<I...>) {
        return logger.Log(time, vec(I)...);
    }

    template <typename T, size_t... I1, typename U, size_t... I2>
    void LogVector2Impl(frc::CSVLogFile& logger, const T& vec1,
                        std::index_sequence<I1...>, const U& vec2,
                        std::index_sequence<I2...>) {
        return logger.Log(vec1(I1)..., vec2(I2)...);
    }

    template <typename T, size_t... I1, typename U, size_t... I2>
    void LogVector2WithTimeImpl(frc::CSVLogFile& logger, units::second_t time,
                                const T& vec1, std::index_sequence<I1...>,
                                const U& vec2, std::index_sequence<I2...>) {
        return logger.Log(time, vec1(I1)..., vec2(I2)...);
    }

    template <int Rows, typename Indices = std::make_index_sequence<Rows>>
    void LogVector(frc::CSVLogFile& logger,
                   const Eigen::Vector<double, Rows>& vec) {
        return LogVectorImpl(logger, vec, Indices{});
    }

    template <int Rows, typename Indices = std::make_index_sequence<Rows>>
    void LogVectorWithTime(frc::CSVLogFile& logger, units::second_t time,
                           const Eigen::Vector<double, Rows>& vec) {
        return LogVectorWithTimeImpl(logger, time, vec, Indices{});
    }

    template <int Rows1, int Rows2,
              typename Indices1 = std::make_index_sequence<Rows1>,
              typename Indices2 = std::make_index_sequence<Rows2>>
    void LogVector2(frc::CSVLogFile& logger,
                    const Eigen::Vector<double, Rows1>& vec1,
                    const Eigen::Vector<double, Rows2>& vec2) {
        return LogVector2Impl(logger, vec1, Indices1{}, vec2, Indices2{});
    }

    template <int Rows1, int Rows2,
              typename Indices1 = std::make_index_sequence<Rows1>,
              typename Indices2 = std::make_index_sequence<Rows2>>
    void LogVector2WithTime(frc::CSVLogFile& logger, units::second_t time,
                            const Eigen::Vector<double, Rows1>& vec1,
                            const Eigen::Vector<double, Rows2>& vec2) {
        return LogVector2WithTimeImpl(logger, time, vec1, Indices1{}, vec2,
                                      Indices2{});
    }
};

}  // namespace frc3512
