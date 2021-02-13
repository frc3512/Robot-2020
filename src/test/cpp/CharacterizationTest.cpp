// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include <string>
#include <thread>

#include <fmt/core.h>
#include <frc/simulation/DriverStationSim.h>
#include <gtest/gtest.h>

#include "Robot.hpp"
#include "SimulatorTest.hpp"

class CharacterizationTest : public frc3512::SimulatorTest {
public:
    frc3512::Robot robot;

    ~CharacterizationTest() override {
        robot.EndCompetition();
        robotThread.join();
    }

private:
    std::thread robotThread{[&] { robot.StartCompetition(); }};
};

TEST_F(CharacterizationTest, TestData) {
    frc::sim::DriverStationSim ds;

    ds.SetTest(true);
    ds.SetEnabled(true);
    ds.NotifyNewData();

    frc::sim::StepTiming(3_s);
    EXPECT_NE(robot.TestGetEntries()[2], 0);

    ds.SetEnabled(false);
    ds.NotifyNewData();

    frc::sim::StepTiming(20_ms);
}
