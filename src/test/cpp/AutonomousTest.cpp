// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <memory>
#include <string>
#include <vector>

#include <frc/RobotBase.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc/simulation/SimHooks.h>
#include <gtest/gtest.h>

#include "CSVTestUtil.hpp"
#include "Robot.hpp"

class AutonomousTest : public testing::TestWithParam<std::string> {
protected:
    void SetUp() override {
        frc::sim::PauseTiming();
        frc::RunHALInitialization();
        robot = std::make_unique<frc3512::Robot>();
    }

    void TearDown() override { frc::sim::ResumeTiming(); }

    std::unique_ptr<frc3512::Robot> robot;
    frc::sim::DriverStationSim ds;
};

TEST_P(AutonomousTest, RunMode) {
    using namespace std::literals;

    frc::sim::DriverStationSim ds;

    robot->m_autonSelector.SetMode(GetParam());

    std::thread robotThread{[&] { robot->StartCompetition(); }};

    ds.SetAutonomous(true);
    ds.SetEnabled(true);
    ds.NotifyNewData();

    frc::sim::StepTiming(15_s);

    ds.SetEnabled(false);
    ds.NotifyNewData();

    frc::sim::StepTiming(20_ms);

    robot->EndCompetition();
    robotThread.join();

    frc3512::AddPrefixToCSVs("AutonomousTest "s + GetParam());
}

INSTANTIATE_TEST_SUITE_P(AutonomousTests, AutonomousTest, [] {
    frc3512::Robot robot;
    std::vector<std::string> names;
    for (int i = 0; i < robot.m_autonSelector.Size(); ++i) {
        names.emplace_back(robot.m_autonSelector.GetName(i));
    }
    return testing::ValuesIn(names);
}());
