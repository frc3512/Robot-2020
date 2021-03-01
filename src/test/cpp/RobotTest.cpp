// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include <thread>
#include <vector>

#include <fmt/format.h>
#include <frc/simulation/DriverStationSim.h>
#include <gtest/gtest.h>

#include "Robot.hpp"
#include "SimulatorTest.hpp"

// Simulate conservation of angular momentum when a ball goes through the
// shooter.
//
// Ball state vector has position and angular velocity (MoI is a constant). Keep
// a circular buffer of them in the Robot class. When one moves into range of
// the flywheel, start trasferring angular momentum from the flywheel to it via
// conservation of angular momentum. Or maybe try sum of torques?
//
// Instantly reduce flywheel angular velocity according to the following
// after a ball contacts it, then remove the ball from the queue.
//
// omega_new = J_flywheel / (J_flywheel + J_ball) * omega_old

class RobotTest : public frc3512::SimulatorTest {
public:
    frc3512::Robot robot;

    RobotTest() {
        ds.SetAutonomous(false);
        ds.SetEnabled(true);
        ds.NotifyNewData();
    }

    ~RobotTest() override {
        robot.EndCompetition();
        robotThread.join();
    }

private:
    std::thread robotThread{[&] { robot.StartCompetition(); }};
    frc::sim::DriverStationSim ds;
};

std::vector<int> Range(int start, int end, int step = 1) {
    std::vector<int> ret;
    for (int i = start; i < end; i += step) {
        ret.emplace_back(i);
    }
    return ret;
}

std::vector<int> Range(int end) { return Range(0, end); }

// Make sure robot initializes
TEST_F(RobotTest, Init) {}

// Verify shooter times out when the number of balls isn't specified and when
// attempting to shoot more balls than the robot contains
TEST_F(RobotTest, DISABLED_ShootTimeout) {
    for (int ballsToShoot : {-1, 1, 2, 3, 4, 5}) {
        // 0 to "ballsToShoot - 1"
        for (int ballsInRobot : Range(std::max(0, ballsToShoot))) {
            for (int i = 0; i < ballsInRobot; ++i) {
                robot.intakeSim.AddBall();
            }

            robot.Shoot(ballsToShoot);

            // Wait for flywheel to reach initial goal before starting timeout
            while (!robot.flywheel.AtGoal()) {
                frc::sim::StepTiming(5_ms);
            }

            // Wait for shooter to finish and expect a timeout
            auto msg =
                fmt::format("\twhere ballsToShoot={} and ballsInRobot={}\n",
                            ballsToShoot, ballsInRobot);
            EXPECT_TRUE(robot.IsShooting()) << msg;
            frc::sim::StepTiming(frc3512::Robot::kMaxShootTimeout / 2.0);
            EXPECT_TRUE(robot.IsShooting()) << msg;
            frc::sim::StepTiming(frc3512::Robot::kMaxShootTimeout);
            EXPECT_FALSE(robot.IsShooting()) << msg;

            EXPECT_EQ(robot.intakeSim.NumberOfBalls(), 0u);

            robot.intakeSim.RemoveAllBalls();
        }
    }
}

// Verify all queued balls are shot before the timeout
TEST_F(RobotTest, DISABLED_ShootNoTimeout) {
    for (int ballsToShoot : {1, 2, 3, 4, 5}) {
        for (int i = 0; i < ballsToShoot; ++i) {
            robot.intakeSim.AddBall();
        }

        robot.Shoot(ballsToShoot);

        // Wait for flywheel to reach initial goal before starting timeout
        while (!robot.flywheel.AtGoal()) {
            frc::sim::StepTiming(5_ms);
        }

        // Wait for shooter to finish and expect no timeout
        auto msg = fmt::format("\twhere ballsToShoot={}\n", ballsToShoot);
        EXPECT_TRUE(robot.IsShooting()) << msg;
        frc::sim::StepTiming(frc3512::Robot::kMaxShootTimeout - 20_ms);
        EXPECT_FALSE(robot.IsShooting()) << msg;

        EXPECT_EQ(robot.intakeSim.NumberOfBalls(), 0u);

        robot.intakeSim.RemoveAllBalls();
    }
}
