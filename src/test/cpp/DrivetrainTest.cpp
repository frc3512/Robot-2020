// Copyright (c) 2019-2020 FRC Team 3512. All Rights Reserved.

#include <cmath>

#include <frc/Notifier.h>
#include <frc/simulation/SimHooks.h>
#include <frc2/Timer.h>
#include <gtest/gtest.h>
#include <units/math.h>

#include "CSVTestUtil.hpp"
#include "Constants.hpp"
#include "controllers/DrivetrainController.hpp"
#include "subsystems/Drivetrain.hpp"

namespace {
class DrivetrainTest : public testing::Test {
protected:
    void SetUp() override { frc::sim::PauseTiming(); }

    void TearDown() override { frc::sim::ResumeTiming(); }
};
}  // namespace

TEST_F(DrivetrainTest, ReachesReferenceStraight) {
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

    frc3512::AddPrefixToCSVs("DrivetrainTest Straight");

    EXPECT_TRUE(drivetrain.AtGoal());
}

TEST_F(DrivetrainTest, ReachesReferenceCurve) {
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

    frc3512::AddPrefixToCSVs("DrivetrainTest Curve");

    EXPECT_TRUE(drivetrain.AtGoal());
}

TEST(DrivetrainTest, ReachesReferenceCorrectWithGlobalY) {
    static constexpr bool kIdealModel = false;

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
        frc::sim::StepTiming(frc3512::Constants::kDt);
    }

    frc3512::SubsystemBase::RunAllDisabledInit();
    frc3512::ControllerSubsystemBase::Disable();

    using State = frc3512::DrivetrainController::State;
    using GlobalOutput = frc3512::DrivetrainController::GlobalOutput;

    auto oldXhat = drivetrain.GetNextXhat();

    Eigen::Matrix<double, 10, 1> x;
    x << 5.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    double xDiff1 = std::abs(oldXhat(State::kX) - x(State::kX));
    double yDiff1 = std::abs(oldXhat(State::kY) - x(State::kY));

    Eigen::Matrix<double, 2, 1> y =
        frc3512::DrivetrainController::GlobalMeasurementModel(
            x, Eigen::Matrix<double, 2, 1>::Zero());

    // Add measurement noise
    if constexpr (!kIdealModel) {
        y += frc::MakeWhiteNoiseVector(0.05, 0.05);
    }

    int64_t latency = -500;
    auto dataTimestamp =
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count() +
        latency;
    drivetrain.CorrectWithGlobalOutputs(units::meter_t{y(GlobalOutput::kX)},
                                        units::meter_t{y(GlobalOutput::kY)},
                                        dataTimestamp);

    frc::sim::StepTiming(frc3512::Constants::kDt);

    auto newXhat = drivetrain.GetNextXhat();

    double xDiff2 = std::abs(newXhat(State::kX) - x(State::kX));
    double yDiff2 = std::abs(newXhat(State::kY) - x(State::kY));

    RenameCSVs("DrivetrainControllerTest GlobalY", "./Drivetrain ");

    EXPECT_GT(xDiff1, xDiff2);
    EXPECT_GT(yDiff1, yDiff2);
}
