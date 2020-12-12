// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <string>
#include <vector>

#include <fmt/core.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc/simulation/SimHooks.h>
#include <gtest/gtest.h>

#include "CSVTestUtil.hpp"
#include "Robot.hpp"

class AutonomousTest : public testing::TestWithParam<std::string> {
protected:
    void SetUp() override { frc::sim::PauseTiming(); }

    void TearDown() override { frc::sim::ResumeTiming(); }
};

TEST_P(AutonomousTest, RunMode) {
    using namespace std::literals;

    fmt::print("[          ] where GetParam() = \"{}\"\n", GetParam());

    frc::sim::DriverStationSim ds;

    {
        frc3512::Robot robot;

        std::thread robotThread{[&] { robot.StartCompetition(); }};

        robot.SelectAutonomous(GetParam());

        ds.SetAutonomous(true);
        ds.SetEnabled(true);
        ds.NotifyNewData();

        frc::sim::StepTiming(15_s);

        robot.ExpectAutonomousEndConds();

        ds.SetEnabled(false);
        ds.NotifyNewData();

        frc::sim::StepTiming(20_ms);

        robot.EndCompetition();
        robotThread.join();
    }

    frc3512::AddPrefixToCSVs("AutonomousTest "s + GetParam());
}

INSTANTIATE_TEST_SUITE_P(AutonomousTests, AutonomousTest, [] {
    frc3512::Robot robot;
    return testing::ValuesIn(robot.GetAutonomousNames());
}());
