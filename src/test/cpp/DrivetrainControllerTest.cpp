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

void RunSimulation(
    frc3512::DrivetrainController& controller,
    Eigen::Matrix<double, 10, 1> x = Eigen::Matrix<double, 10, 1>::Zero()) {
    using frc3512::Constants::kDt;

    frc::Pose2d initialPose{
        units::meter_t{x(frc3512::DrivetrainController::State::kX)},
        units::meter_t{x(frc3512::DrivetrainController::State::kY)},
        units::radian_t{x(frc3512::DrivetrainController::State::kHeading)}};
    controller.Reset(initialPose, initialPose);

    Eigen::Matrix<double, 2, 1> u = Eigen::Matrix<double, 2, 1>::Zero();

    frc::sim::RoboRioSim roboRIO{0};

    auto currentTime = 0_s;
    while (currentTime < 10_s) {
        auto dt = kDt;
        if constexpr (!kIdealModel) {
            dt += units::second_t{frc::MakeWhiteNoiseVector(0.001)(0)};
        }

        // Update drivetrain controller
        Eigen::Matrix<double, 3, 1> y =
            frc3512::DrivetrainController::LocalMeasurementModel(
                x, Eigen::Matrix<double, 2, 1>::Zero());
        // Add measurement noise to drivetrain controller
        if constexpr (!kIdealModel) {
            y += frc::MakeWhiteNoiseVector(0.0001, 0.005, 0.005);
        }
        controller.SetMeasuredLocalOutputs(
            units::radian_t{y(0)}, units::meter_t{y(1)}, units::meter_t{y(2)});
        controller.Update(kDt, currentTime);

        u = controller.GetInputs();

        // Account for battery voltage drop due to current draw for the turret
        if constexpr (!kIdealModel) {
            constexpr auto Vbat = 12_V;
            constexpr auto Rbat = 0.03_Ohm;
            constexpr auto r = 0.0746125_m;  // Drivetrain wheel radius

            using Input = frc3512::DrivetrainController::Input;
            using State = frc3512::DrivetrainController::State;
            constexpr auto motors = frc::DCMotor::NEO(2);
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
}

TEST(DrivetrainControllerTest, ReachesReferenceStraight) {
    using frc3512::Constants::kDt;

    frc::sim::RoboRioSim roboRIO{0};

    frc::Pose2d initialPose{12.65_m, 5.800_m - 0.343_m,
                            units::radian_t{wpi::math::pi}};

    Eigen::Matrix<double, 10, 1> x0 = Eigen::Matrix<double, 10, 1>::Zero();
    x0(frc3512::DrivetrainController::State::kX) =
        initialPose.Translation().X().to<double>();
    x0(frc3512::DrivetrainController::State::kY) =
        initialPose.Translation().Y().to<double>();
    x0(frc3512::DrivetrainController::State::kHeading) =
        initialPose.Rotation().Radians().to<double>();

    frc3512::DrivetrainController controller;
    controller.SetWaypoints(
        initialPose, {},
        frc::Pose2d(12.65_m - 0.9398_m - 0.5_m, 5.800_m - 0.343_m,
                    units::radian_t{wpi::math::pi}));
    controller.Enable();

    RunSimulation(controller, x0);

    RenameCSVs("DrivetrainControllerTest Straight", "./Drivetrain ");

    EXPECT_TRUE(controller.AtGoal());
}

TEST(DrivetrainControllerTest, ReachesReferenceCurve) {
    using frc3512::Constants::kDt;

    frc::sim::RoboRioSim roboRIO{0};

    frc3512::DrivetrainController controller;
    controller.SetWaypoints(frc::Pose2d(0_m, 0_m, 0_rad), {},
                            frc::Pose2d(4.8768_m, 2.7432_m, 0_rad));
    controller.Enable();

    RunSimulation(controller);

    RenameCSVs("DrivetrainControllerTest Curve", "./Drivetrain ");

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
