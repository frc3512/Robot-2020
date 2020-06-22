// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <fstream>

#include <frc/system/plant/DCMotor.h>
#include <gtest/gtest.h>
#include <simulation/RoboRioSim.h>
#include <units/units.h>

#include "Constants.hpp"
#include "RenameCSVs.hpp"
#include "controllers/FlywheelController.hpp"

static constexpr bool kIdealModel = false;

TEST(FlywheelControllerTest, ReachesGoal) {
    using frc3512::Constants::kDt;

    frc::sim::RoboRioSim roboRIO{0};

    frc3512::FlywheelController controller;
    controller.Reset();
    controller.Enable();

    controller.SetMeasuredAngularVelocity(0_rad_per_s);
    controller.SetGoal(500.0_rad_per_s);

    Eigen::Matrix<double, 1, 1> x = Eigen::Matrix<double, 1, 1>::Zero();

    Eigen::Matrix<double, 1, 1> u = Eigen::Matrix<double, 1, 1>::Zero();

    auto currentTime = 0_s;
    while (currentTime < 10_s) {
        auto dt = kDt;

        // Add scheduling jitter
        if constexpr (!kIdealModel) {
            dt += units::second_t{frc::MakeWhiteNoiseVector(0.001)(0)};
        }

        Eigen::Matrix<double, 1, 1> y = x;

        // Add measurement noise
        if constexpr (!kIdealModel) {
            y += frc::MakeWhiteNoiseVector<1>({50.0});
        }

        controller.SetMeasuredAngularVelocity(
            units::radians_per_second_t{y(0)});
        controller.SetMeasuredInputs(units::volt_t{u(0)});
        controller.Update(kDt, currentTime);

        u = controller.GetInputs();

        // Account for battery voltage drop due to current draw
        if constexpr (!kIdealModel) {
            constexpr auto Vbat = 12_V;
            constexpr auto Rbat = 0.03_Ohm;

            constexpr auto motors = frc::DCMotor::NEO(2);
            units::ampere_t load = motors.Current(
                units::radians_per_second_t{
                    x(0) / frc3512::FlywheelController::kGearRatio},
                units::volt_t{u(0)});
            units::volt_t vLoaded = Vbat - load * Rbat - load * Rbat;
            double dsVoltage =
                vLoaded.to<double>() + frc::MakeWhiteNoiseVector(0.1)(0);
            roboRIO.SetVInVoltage(dsVoltage);

            u *= dsVoltage / 12.0;
        }

        x = controller.GetPlant().CalculateX(x, u, dt);
        currentTime += dt;
    }

    RenameCSVs("FlywheelControllerTest", "./Flywheel ");

    EXPECT_TRUE(controller.AtGoal());
}
