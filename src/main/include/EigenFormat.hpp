// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Eigen/Core>
#include <fmt/format.h>

template <int Rows, int Cols, int... Args>
struct fmt::formatter<Eigen::Matrix<double, Rows, Cols, Args...>> {
    char presentation = 'f';

    constexpr auto parse(fmt::format_parse_context& ctx) {
        auto it = ctx.begin(), end = ctx.end();
        if (it != end && (*it == 'f' || *it == 'e')) {
            presentation = *it++;
        }

        if (it != end && *it != '}') {
            throw fmt::format_error("invalid format");
        }

        return it;
    }

    template <typename FormatContext>
    auto format(const Eigen::Matrix<double, Rows, Cols, Args...>& mat,
                FormatContext& ctx) {
        auto out = ctx.out();
        for (int i = 0; i < Rows; ++i) {
            for (int j = 0; j < Cols; ++j) {
                out = fmt::format_to(out, "  {:f}", mat(i, j));
            }

            if (i < Rows - 1) {
                out = fmt::format_to(out, "\n");
            }
        }

        return out;
    }
};
