// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <string>

#include <fmt/format.h>
#include <frc/simulation/SimHooks.h>
#include <gtest/gtest.h>

#include "SetCurrentPath.hpp"

namespace frc3512 {

/**
 * This fixture pauses/unpauses simulation time and sets the test CSV current
 * path automatically. The current path is reset at the end of each test.
 *
 * The current path is constructed as the test suite name then the test name.
 */
class SimulatorTest : public testing::Test {
protected:
    SetCurrentPath testPath = [] {
        auto testInfo = testing::UnitTest::GetInstance()->current_test_info();
        return SetCurrentPath{fmt::format("{}/{}", testInfo->test_suite_name(),
                                          testInfo->name())};
    }();

    SimulatorTest() { frc::sim::PauseTiming(); }

    ~SimulatorTest() override { frc::sim::ResumeTiming(); }
};

}  // namespace frc3512
