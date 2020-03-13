// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <fstream>

#include <frc/system/plant/DCMotor.h>
#include <gtest/gtest.h>
#include <simulation/RoboRioSim.h>
#include <units/units.h>
#include <wpi/circular_buffer.h>

#include "Constants.hpp"
#include "RenameCSVs.hpp"
#include "controllers/FlywheelController.hpp"

TEST(FlywheelControllerTest, ReachesGoal) {
    using frc3512::Constants::kDt;

    frc::sim::RoboRioSim roboRIO{0};

    frc3512::FlywheelController controller{{80.0}, {12.0}, kDt};
    controller.Reset();
    controller.Enable();

    controller.SetMeasuredAngularVelocity(0_rad_per_s);
    controller.SetGoal(500.0_rad_per_s);

    Eigen::Matrix<double, 1, 1> x = Eigen::Matrix<double, 1, 1>::Zero();

    auto currentTime = 0_s;
    while (currentTime < 10_s) {
        auto dt = kDt + units::second_t{frc::MakeWhiteNoiseVector(0.001)(0)};
        Eigen::Matrix<double, 1, 1> noise =
            frc::MakeWhiteNoiseVector<1>({0.08});

        controller.SetMeasuredAngularVelocity(
            units::radians_per_second_t{x(0) + noise(0)});

        controller.Update(kDt, currentTime);
        currentTime += dt;

        constexpr auto Vbat = 12_V;
        constexpr auto Rbat = 0.03_Ohm;
        Eigen::Matrix<double, 1, 1> u = controller.GetInputs();

        // Account for battery voltage drop due to current draw
        constexpr auto motors = frc::DCMotor::NEO(2);
        units::ampere_t load = motors.Current(
            units::radians_per_second_t{
                x(0) / frc3512::FlywheelController::kGearRatio},
            units::volt_t{u(0)});
        units::volt_t vLoaded = Vbat - load * Rbat - load * Rbat;
        double dsVoltage =
            vLoaded.to<double>() + frc::MakeWhiteNoiseVector(0.1)(0);
        roboRIO.SetVInVoltage(dsVoltage);
        Eigen::Matrix<double, 1, 1> trueU = u;
        trueU *= dsVoltage / 12.0;

        x = controller.GetStates();
    }

    RenameCSVs("FlywheelControllerTest", "./Flywheel ");

    EXPECT_TRUE(controller.AtGoal());
}
