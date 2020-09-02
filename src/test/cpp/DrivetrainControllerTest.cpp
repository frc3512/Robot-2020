// Copyright (c) 2019-2020 FRC Team 3512. All Rights Reserved.

#include <cmath>

#include <frc/StateSpaceUtil.h>
#include <frc/system/plant/DCMotor.h>
#include <gtest/gtest.h>
#include <simulation/RoboRioSim.h>
#include <units/units.h>

#include "Constants.hpp"
#include "RenameCSVs.hpp"
#include "controllers/DrivetrainController.hpp"

static constexpr bool kIdealModel = false;

TEST(DrivetrainControllerTest, ReachesReference) {
    using frc3512::Constants::kDt;

    frc::sim::RoboRioSim roboRIO{0};

    frc::Pose2d initialPose{12.65_m, 5.800_m - 0.343_m,
                            units::radian_t{wpi::math::pi}};

    frc3512::DrivetrainController controller;
    controller.Reset(initialPose, initialPose);
    controller.SetOpenLoop(false);
    controller.Enable();

    Eigen::Matrix<double, 10, 1> x = Eigen::Matrix<double, 10, 1>::Zero();
    x(frc3512::DrivetrainController::State::kX) =
        initialPose.Translation().X().to<double>();
    x(frc3512::DrivetrainController::State::kY) =
        initialPose.Translation().Y().to<double>();
    x(frc3512::DrivetrainController::State::kHeading) =
        initialPose.Rotation().Radians().to<double>();

    Eigen::Matrix<double, 2, 1> u = Eigen::Matrix<double, 2, 1>::Zero();

    auto currentTime = 0_s;

    controller.Update(kDt, currentTime);

    controller.SetWaypoints(
        frc::Pose2d(12.65_m, 5.800_m - 0.343_m, units::radian_t{wpi::math::pi}),
        {},
        frc::Pose2d(12.65_m - 0.9398_m - 0.5_m, 5.800_m - 0.343_m,
                    units::radian_t{wpi::math::pi}));
    while (currentTime < 10_s) {
        auto dt = kDt;

        // Add scheduling jitter
        if constexpr (!kIdealModel) {
            dt += units::second_t{frc::MakeWhiteNoiseVector(0.001)(0)};
        }

        Eigen::Matrix<double, 3, 1> y =
            frc3512::DrivetrainController::LocalMeasurementModel(
                x, Eigen::Matrix<double, 2, 1>::Zero());

        // Add measurement noise
        if constexpr (!kIdealModel) {
            y += frc::MakeWhiteNoiseVector(0.0001, 0.005, 0.005);
        }

        controller.SetMeasuredLocalOutputs(
            units::radian_t{y(0)}, units::meter_t{y(1)}, units::meter_t{y(2)});
        controller.SetMeasuredInputs(units::volt_t{u(0)}, units::volt_t{u(1)});
        controller.Update(kDt, currentTime);

        u = controller.GetInputs();

        // Account for battery voltage drop due to current draw
        if constexpr (!kIdealModel) {
            constexpr auto Vbat = 12_V;
            constexpr auto Rbat = 0.03_Ohm;
            constexpr auto r = 0.0746125_m;  // Wheel radius

            using Input = frc3512::DrivetrainController::Input;
            using State = frc3512::DrivetrainController::State;
            constexpr auto motors = frc::DCMotor::MiniCIM(3);
            units::ampere_t loadIleft =
                motors.Current(
                    units::meters_per_second_t{x(State::kLeftVelocity)} / r *
                        1_rad,
                    units::volt_t{u(Input::kLeftVoltage)}) *
                wpi::sgn(u(Input::kLeftVoltage));
            units::ampere_t loadIright =
                motors.Current(
                    units::meters_per_second_t{x(State::kRightVelocity)} / r *
                        1_rad,
                    units::volt_t{u(Input::kRightVoltage)}) *
                wpi::sgn(u(Input::kRightVoltage));
            units::volt_t vLoaded = Vbat - loadIleft * Rbat - loadIright * Rbat;
            double dsVoltage =
                vLoaded.to<double>() + frc::MakeWhiteNoiseVector(0.1)(0);
            roboRIO.SetVInVoltage(dsVoltage);

            u *= dsVoltage / 12.0;
        }

        x = frc::RungeKutta(frc3512::DrivetrainController::Dynamics, x, u, dt);
        currentTime += dt;
    }

    RenameCSVs("DrivetrainControllerTest", "./Drivetrain ");

    EXPECT_TRUE(controller.AtGoal());
}

TEST(DrivetrainControllerTest, CorrectsTowardGlobalY) {
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
