// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <string_view>

#include <fmt/format.h>
#include <frc/simulation/SimHooks.h>
#include <gtest/gtest.h>

#include "SetCurrentPath.hpp"

namespace frc3512 {

/**
 * This fixture pauses/unpauses simulation time and sets the test CSV current
 * path automatically. The current path is reset at the end of each test.
 *
 * The current path is constructed as the test suite name, the test name with
 * the parameterization index stripped, then the stringified parameter.
 */
template <typename T>
class SimulatorTestWithParam : public testing::TestWithParam<T> {
protected:
    SetCurrentPath testPath = [=] {
        auto testInfo = testing::UnitTest::GetInstance()->current_test_info();
        std::string_view name = testInfo->name();

        // Name substring removes suffixed parameterization index
        return SetCurrentPath{
            fmt::format("{}/{}/{}", testInfo->test_suite_name(),
                        name.substr(0, name.rfind("/")), this->GetParam())};
    }();

    SimulatorTestWithParam() { frc::sim::PauseTiming(); }

    ~SimulatorTestWithParam() override { frc::sim::ResumeTiming(); }
};

}  // namespace frc3512
