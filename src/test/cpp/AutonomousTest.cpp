// Copyright (c) FRC Team 3512. All Rights Reserved.

#include <string>
#include <thread>

#include <fmt/core.h>
#include <frc/simulation/DriverStationSim.h>
#include <gtest/gtest.h>

#include "Robot.hpp"
#include "SetCurrentPath.hpp"
#include "SimulatorTestWithParam.hpp"

class AutonomousTest : public frc3512::SimulatorTestWithParam<std::string> {
public:
    frc3512::Robot robot;

    ~AutonomousTest() override {
        robot.EndCompetition();
        robotThread.join();
    }

private:
    std::thread robotThread{[&] { robot.StartCompetition(); }};
};

TEST_P(AutonomousTest, Run) {
    fmt::print("[          ] where GetParam() = \"{}\"\n", GetParam());

    frc::sim::DriverStationSim ds;

    robot.SelectAutonomous(GetParam());

    ds.SetAutonomous(true);
    ds.SetEnabled(true);
    ds.NotifyNewData();

    frc::sim::StepTiming(robot.SelectedAutonomousDuration());

    robot.ExpectAutonomousEndConds();

    ds.SetEnabled(false);
    ds.NotifyNewData();

    frc::sim::StepTiming(20_ms);
}

INSTANTIATE_TEST_SUITE_P(AutonomousTests, AutonomousTest, [] {
    frc3512::SetCurrentPath testPath{
        "AutonomousTests/AutonomousTest/Parameterization"};
    frc3512::Robot robot;
    return testing::ValuesIn(robot.GetAutonomousNames());
}());
