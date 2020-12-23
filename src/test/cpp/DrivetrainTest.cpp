// Copyright (c) 2019-2020 FRC Team 3512. All Rights Reserved.

#include <frc/Notifier.h>
#include <frc/simulation/SimHooks.h>
#include <gtest/gtest.h>

#include "Constants.hpp"
#include "logging/CSVUtil.hpp"
#include "subsystems/Drivetrain.hpp"

namespace {
class DrivetrainTest : public testing::Test {
protected:
    void SetUp() override { frc::sim::PauseTiming(); }

    void TearDown() override { frc::sim::ResumeTiming(); }
};
}  // namespace

TEST_F(DrivetrainTest, ReachesReferenceStraight) {
    {
        frc3512::Drivetrain drivetrain;

        frc3512::SubsystemBase::RunAllAutonomousInit();
        frc::Notifier controllerPeriodic{[&] {
            drivetrain.AutonomousPeriodic();
            drivetrain.ControllerPeriodic();
        }};
        controllerPeriodic.StartPeriodic(frc3512::Constants::kDt);

        frc::Pose2d initialPose{12.65_m, 5.800_m - 0.343_m,
                                units::radian_t{wpi::math::pi}};

        drivetrain.Reset(initialPose);
        drivetrain.SetWaypoints(
            initialPose, {},
            frc::Pose2d(12.65_m - 0.9398_m - 0.5_m, 5.800_m - 0.343_m,
                        units::radian_t{wpi::math::pi}));

        frc::sim::StepTiming(10_s);

        frc3512::SubsystemBase::RunAllDisabledInit();

        EXPECT_TRUE(drivetrain.AtGoal());
    }

    frc3512::AddPrefixToCSVs("DrivetrainTest Straight");
}

TEST_F(DrivetrainTest, ReachesReferenceCurve) {
    {
        frc3512::Drivetrain drivetrain;

        frc3512::SubsystemBase::RunAllAutonomousInit();
        frc::Notifier controllerPeriodic{
            [&] { drivetrain.ControllerPeriodic(); }};
        controllerPeriodic.StartPeriodic(frc3512::Constants::kDt);

        frc::Pose2d initialPose{0_m, 0_m, 0_rad};

        drivetrain.Reset(initialPose);
        drivetrain.SetWaypoints(initialPose, {},
                                frc::Pose2d(4.8768_m, 2.7432_m, 0_rad));

        frc::sim::StepTiming(10_s);

        frc3512::SubsystemBase::RunAllDisabledInit();

        EXPECT_TRUE(drivetrain.AtGoal());
    }

    frc3512::AddPrefixToCSVs("DrivetrainTest Curve");
}

TEST_F(DrivetrainTest, ReachesReferenceOffsetCurve) {
    {
        frc3512::Drivetrain drivetrain;

        frc3512::SubsystemBase::RunAllAutonomousInit();
        frc::Notifier controllerPeriodic{
            [&] { drivetrain.ControllerPeriodic(); }};
        controllerPeriodic.StartPeriodic(frc3512::Constants::kDt);

        frc::Pose2d initialPose{5_m, 2_m, 0_rad};

        drivetrain.Reset(initialPose);
        drivetrain.SetWaypoints(initialPose, {},
                                frc::Pose2d(9.8768_m, 4.7432_m, 0_rad));

        frc::sim::StepTiming(10_s);

        frc3512::SubsystemBase::RunAllDisabledInit();

        EXPECT_TRUE(drivetrain.AtGoal());
    }

    frc3512::AddPrefixToCSVs("DrivetrainTest OffsetCurve");
}
