// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <utility>

#include <Eigen/Core>
#include <frc/logging/CSVLogFile.h>
#include <units/time.h>

namespace frc3512 {
namespace detail {
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
}  // namespace detail

template <int Rows, typename Indices = std::make_index_sequence<Rows>>
void LogVector(frc::CSVLogFile& logger,
               const Eigen::Matrix<double, Rows, 1>& vec) {
    return detail::LogVectorImpl(logger, vec, Indices{});
}

template <int Rows, typename Indices = std::make_index_sequence<Rows>>
void LogVectorWithTime(frc::CSVLogFile& logger, units::second_t time,
                       const Eigen::Matrix<double, Rows, 1>& vec) {
    return detail::LogVectorWithTimeImpl(logger, time, vec, Indices{});
}

template <int Rows1, typename Indices1 = std::make_index_sequence<Rows1>,
          int Rows2, typename Indices2 = std::make_index_sequence<Rows2>>
void LogVector2(frc::CSVLogFile& logger,
                const Eigen::Matrix<double, Rows1, 1>& vec1,
                const Eigen::Matrix<double, Rows2, 1>& vec2) {
    return detail::LogVector2Impl(logger, vec1, Indices1{}, vec2, Indices2{});
}

template <int Rows1, typename Indices1 = std::make_index_sequence<Rows1>,
          int Rows2, typename Indices2 = std::make_index_sequence<Rows2>>
void LogVector2WithTime(frc::CSVLogFile& logger, units::second_t time,
                        const Eigen::Matrix<double, Rows1, 1>& vec1,
                        const Eigen::Matrix<double, Rows2, 1>& vec2) {
    return detail::LogVector2WithTimeImpl(logger, time, vec1, Indices1{}, vec2,
                                          Indices2{});
}
}  // namespace frc3512
