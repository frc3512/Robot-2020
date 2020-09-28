// Copyright (c) 2019-2020 FRC Team 3512. All Rights Reserved.

#include <frc/Notifier.h>
#include <frc/simulation/SimHooks.h>
#include <frc2/Timer.h>
#include <gtest/gtest.h>
#include <units/math.h>

#include "Constants.hpp"
#include "RenameCSVs.hpp"
#include "subsystems/Drivetrain.hpp"

TEST(DrivetrainTest, ReachesReferenceStraight) {
    frc::sim::PauseTiming();

    frc3512::Drivetrain drivetrain;

    frc3512::SubsystemBase::RunAllAutonomousInit();
    frc3512::ControllerSubsystemBase::Enable();

    frc::Pose2d initialPose{12.65_m, 5.800_m - 0.343_m,
                            units::radian_t{wpi::math::pi}};

    drivetrain.SetWaypoints(
        initialPose, {},
        frc::Pose2d(12.65_m - 0.9398_m - 0.5_m, 5.800_m - 0.343_m,
                    units::radian_t{wpi::math::pi}));

    drivetrain.Reset(initialPose);

    frc::Notifier autonomousPeriodic{
        &frc3512::SubsystemBase::RunAllAutonomousPeriodic};
    autonomousPeriodic.StartPeriodic(20_ms);

    frc2::Timer timer;
    timer.Start();
    while (timer.Get() < 10_s) {
        // This noise simulates scheduling jitter. It's clamped to 0 so the next
        // Notifier still triggers.
        auto noise = units::math::min(
            0_s,
            units::second_t{frc::MakeWhiteNoiseVector(0.001)(0)} + 0.001_s);
        frc::sim::StepTiming(frc3512::Constants::kDt + noise);
    }

    frc3512::SubsystemBase::RunAllDisabledInit();
    frc3512::ControllerSubsystemBase::Disable();

    frc::sim::ResumeTiming();

    RenameCSVs("DrivetrainControllerTest Straight", "./Drivetrain ");

    EXPECT_TRUE(drivetrain.AtGoal());
}

TEST(DrivetrainTest, ReachesReferenceCurve) {
    frc::sim::PauseTiming();

    frc3512::Drivetrain drivetrain;

    frc3512::SubsystemBase::RunAllAutonomousInit();
    frc3512::ControllerSubsystemBase::Enable();

    frc::Pose2d initialPose{0_m, 0_m, 0_rad};

    drivetrain.SetWaypoints(initialPose, {},
                            frc::Pose2d(4.8768_m, 2.7432_m, 0_rad));

    drivetrain.Reset(initialPose);

    frc::Notifier autonomousPeriodic{
        &frc3512::SubsystemBase::RunAllAutonomousPeriodic};
    autonomousPeriodic.StartPeriodic(20_ms);

    frc2::Timer timer;
    timer.Start();
    while (timer.Get() < 10_s) {
        // This noise simulates scheduling jitter. It's clamped to 0 so the next
        // Notifier still triggers.
        auto noise = units::math::min(
            0_s,
            units::second_t{frc::MakeWhiteNoiseVector(0.001)(0)} + 0.001_s);
        frc::sim::StepTiming(frc3512::Constants::kDt + noise);
    }

    frc3512::SubsystemBase::RunAllDisabledInit();
    frc3512::ControllerSubsystemBase::Disable();

    frc::sim::ResumeTiming();

    RenameCSVs("DrivetrainControllerTest Curve", "./Drivetrain ");

    EXPECT_TRUE(drivetrain.AtGoal());
}
