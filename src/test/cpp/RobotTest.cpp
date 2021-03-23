// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include <thread>
#include <vector>

#include <fmt/format.h>
#include <frc/Notifier.h>
#include <frc/simulation/DriverStationSim.h>
#include <gtest/gtest.h>

#include "Robot.hpp"
#include "SimulatorTest.hpp"

#define EXPECT_NEAR_UNITS(val1, val2, eps) \
    EXPECT_LE(units::math::abs(val1 - val2), eps)

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
            while (!robot.FlywheelAtGoal()) {
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
        while (!robot.FlywheelAtGoal()) {
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

TEST_F(RobotTest, CalculateDrivetrainInGlobal) {
    // Test that vision calculates the same pose that the drivetrain is reset to
    auto testMeasurement = [&](units::meter_t x, units::meter_t y,
                               units::radian_t theta) {
        robot.drivetrain.Reset(frc::Pose2d{x, y, theta});

        // The vision subsystem only processes data when the robot is enabled
        frc::sim::DriverStationSim::SetEnabled(true);
        frc::sim::DriverStationSim::NotifyNewData();
        frc::sim::StepTiming(0_ms);  // Wait for Notifiers

        // Flush and delay to ensure value propagates to tables and controller
        // updates
        nt::NetworkTableInstance::GetDefault().Flush();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        frc::sim::StepTiming(40_ms);

        auto drivetrainInGlobal =
            robot.turret.GetDrivetrainInGlobalMeasurement();

        fmt::print("drivetrainInGlobal ({}, {}, {})\n",
                   drivetrainInGlobal.X().to<double>(),
                   drivetrainInGlobal.Y().to<double>(),
                   drivetrainInGlobal.Rotation().Radians().to<double>());
        fmt::print("initialPose ({}, {}, {})\n", x.to<double>(), y.to<double>(),
                   theta.to<double>());

        ASSERT_GT(robot.drivetrain.GetPose().X(), 0_m);
        ASSERT_GT(robot.drivetrain.GetPose().Y(), 0_m);
        EXPECT_NEAR_UNITS(drivetrainInGlobal.X(), x, 0.2_m);
        EXPECT_NEAR_UNITS(drivetrainInGlobal.Y(), y, 0.2_m);
    };

    testMeasurement(12.89_m, 2.41_m, units::radian_t{wpi::numbers::pi});
}
