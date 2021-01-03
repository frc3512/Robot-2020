// Copyright (c) 2019-2021 FRC Team 3512. All Rights Reserved.

#include <frc2/Timer.h>
#include <gtest/gtest.h>
#include <units/length.h>

#include "Constants.hpp"
#include "SimulatorTest.hpp"
#include "controllers/DrivetrainController.hpp"

class DrivetrainControllerTest : public frc3512::SimulatorTest {};

TEST_F(DrivetrainControllerTest, CorrectsTowardGlobalY) {
    static constexpr bool kIdealModel = false;

    using frc3512::Constants::kDt;

    frc3512::DrivetrainController controller;

    controller.AddTrajectory(frc::Pose2d(0_m, 0_m, 0_rad), {},
                             frc::Pose2d(4.8768_m, 2.7432_m, 0_rad));

    // Ensure error covariance is nonzero
    for (units::second_t t = 0_s; t < 10_s; t += kDt) {
        controller.Predict(Eigen::Matrix<double, 2, 1>::Zero(), kDt);
    }

    Eigen::Matrix<double, 7, 1> x;
    x << 5.0, 5.0, 1.0, 1.0, 1.0, 1.0, 1.0;

    Eigen::Matrix<double, 3, 1> localY =
        frc3512::DrivetrainController::LocalMeasurementModel(
            x, Eigen::Matrix<double, 2, 1>::Zero());
    Eigen::Matrix<double, 2, 1> globalY =
        frc3512::DrivetrainController::GlobalMeasurementModel(
            x, Eigen::Matrix<double, 2, 1>::Zero());

    // Add measurement noise
    if constexpr (!kIdealModel) {
        localY += frc::MakeWhiteNoiseVector(0.0001, 0.005, 0.005);
        globalY += frc::MakeWhiteNoiseVector(0.05, 0.05);
    }

    using State = frc3512::DrivetrainController::State;
    using GlobalOutput = frc3512::DrivetrainController::GlobalOutput;

    auto xHat = controller.GetStates();
    double xDiff1 = std::abs(xHat(State::kX) - x(State::kX));
    double yDiff1 = std::abs(xHat(State::kY) - x(State::kY));

    controller.Update(localY, frc3512::Constants::kDt);
    controller.CorrectWithGlobalOutputs(
        units::meter_t{globalY(GlobalOutput::kX)},
        units::meter_t{globalY(GlobalOutput::kY)},
        frc2::Timer::GetFPGATimestamp());

    xHat = controller.GetStates();
    double xDiff2 = std::abs(xHat(State::kX) - x(State::kX));
    double yDiff2 = std::abs(xHat(State::kY) - x(State::kY));

    EXPECT_GT(xDiff1, xDiff2);
    EXPECT_GT(yDiff1, yDiff2);
}
