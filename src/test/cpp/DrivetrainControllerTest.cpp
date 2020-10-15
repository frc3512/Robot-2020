// Copyright (c) 2019-2020 FRC Team 3512. All Rights Reserved.

#include <gtest/gtest.h>
#include <units/length.h>

#include "Constants.hpp"
#include "controllers/DrivetrainController.hpp"

TEST(DrivetrainControllerTest, CorrectsTowardGlobalY) {
    static constexpr bool kIdealModel = false;

    using frc3512::Constants::kDt;

    frc3512::DrivetrainController controller;
    controller.SetOpenLoop(false);
    controller.Enable();

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
