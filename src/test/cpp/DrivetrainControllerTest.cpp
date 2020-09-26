// Copyright (c) 2019-2020 FRC Team 3512. All Rights Reserved.

#include <cmath>

#include <frc/StateSpaceUtil.h>
#include <frc/simulation/JoystickSim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/simulation/SimHooks.h>
#include <frc/system/plant/DCMotor.h>
#include <frc2/Timer.h>
#include <gtest/gtest.h>
#include <units/current.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include "Constants.hpp"
#include "RenameCSVs.hpp"
#include "controllers/DrivetrainController.hpp"
#include "subsystems/Drivetrain.hpp"

TEST(DrivetrainControllerTest, ReachesReferenceStraight) {
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

TEST(DrivetrainControllerTest, ReachesReferenceCurve) {
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

TEST(DrivetrainControllerTest, CorrectsTowardGlobalY) {
    static constexpr bool kIdealModel = false;

    using frc3512::Constants::kDt;

    frc3512::DrivetrainController controller;
    controller.SetOpenLoop(false);
    controller.Enable();

    controller.SetMeasuredLocalOutputs(0_rad, 0_m, 0_m);
    controller.SetWaypoints(frc::Pose2d(0_m, 0_m, 0_rad), {},
                            frc::Pose2d(4.8768_m, 2.7432_m, 0_rad));

    // Ensure error covariance is nonzero
    for (int i = 0; i < 1000; ++i) {
        controller.Predict(Eigen::Matrix<double, 2, 1>::Zero(), kDt);
    }

    Eigen::Matrix<double, 10, 1> x;
    x << 5.0, 5.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;

    Eigen::Matrix<double, 2, 1> y =
        frc3512::DrivetrainController::GlobalMeasurementModel(
            x, Eigen::Matrix<double, 2, 1>::Zero());

    // Add measurement noise
    if constexpr (!kIdealModel) {
        y += frc::MakeWhiteNoiseVector(0.05, 0.05);
    }

    using GlobalOutput = frc3512::DrivetrainController::GlobalOutput;
    using State = frc3512::DrivetrainController::State;

    auto xHat = controller.GetStates();
    double xDiff1 = std::abs(xHat(State::kX) - x(State::kX));
    double yDiff1 = std::abs(xHat(State::kY) - x(State::kY));

    controller.CorrectWithGlobalOutputs(units::meter_t{y(GlobalOutput::kX)},
                                        units::meter_t{y(GlobalOutput::kY)}, 0);

    xHat = controller.GetStates();
    double xDiff2 = std::abs(xHat(State::kX) - x(State::kX));
    double yDiff2 = std::abs(xHat(State::kY) - x(State::kY));

    EXPECT_GT(xDiff1, xDiff2);
    EXPECT_GT(yDiff1, yDiff2);
}
