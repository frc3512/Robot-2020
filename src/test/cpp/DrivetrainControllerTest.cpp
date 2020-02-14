// Copyright (c) 2019-2020 FRC Team 3512. All Rights Reserved.

#include <cmath>

#include <frc/StateSpaceUtil.h>
#include <frc/system/plant/DCMotor.h>
#include <gtest/gtest.h>
#include <mockdata/RoboRioData.h>
#include <units/units.h>

#include "Constants.hpp"
#include "controllers/DrivetrainController.hpp"

TEST(DrivetrainControllerTest, ReachesReference) {
    using frc3512::Constants::kDt;

    frc3512::DrivetrainController controller{
        {0.0625, 0.125, 10.0, 0.95, 0.95}, {12.0, 12.0}, kDt};
    controller.Reset(frc::Pose2d{0_m, 0_m, 0_rad});
    controller.Enable();

    controller.SetMeasuredLocalOutputs(0_rad, 0_m, 0_m);
    controller.SetWaypoints(
        {frc::Pose2d(0_m, 0_m, 0_rad), frc::Pose2d(4.8768_m, 2.7432_m, 0_rad)});

    Eigen::Matrix<double, 10, 1> trueXhat =
        Eigen::Matrix<double, 10, 1>::Zero();

    auto currentTime = 0_s;
    while (currentTime < 10_s) {
        auto dt = kDt + units::second_t{frc::MakeWhiteNoiseVector(0.001)(0, 0)};

        Eigen::Matrix<double, 3, 1> y =
            frc3512::DrivetrainController::LocalMeasurementModel(
                trueXhat, Eigen::Matrix<double, 2, 1>::Zero()) +
            frc::MakeWhiteNoiseVector(0.0001, 0.005, 0.005);
        controller.SetMeasuredLocalOutputs(units::radian_t{y(0, 0)},
                                           units::meter_t{y(1, 0)},
                                           units::meter_t{y(2, 0)});
        controller.Update(kDt, currentTime);
        currentTime += dt;

        Eigen::Matrix<double, 2, 1> u = controller.GetInputs();

        constexpr auto Vbat = 12_V;
        constexpr auto Rbat = 0.03_Ohm;
        constexpr auto r = 0.0746125_m;  // Wheel radius

        // Account for battery voltage drop due to current draw
        using Input = frc3512::DrivetrainController::Input;
        using State = frc3512::DrivetrainController::State;
        constexpr auto motors = frc::DCMotor::MiniCIM(3);
        units::ampere_t loadIleft = motors.Current(
            units::meters_per_second_t{trueXhat(State::kLeftVelocity, 0)} / r *
                1_rad,
            units::volt_t{u(Input::kLeftVoltage, 0)});
        units::ampere_t loadIright = motors.Current(
            units::meters_per_second_t{trueXhat(State::kRightVelocity, 0)} / r *
                1_rad,
            units::volt_t{u(Input::kRightVoltage, 0)});
        units::volt_t vLoaded = Vbat - loadIleft * Rbat - loadIright * Rbat;
        double dsVoltage =
            vLoaded.to<double>() + frc::MakeWhiteNoiseVector(0.1)(0, 0);
        HALSIM_SetRoboRioVInVoltage(0, dsVoltage);

        Eigen::Matrix<double, 2, 1> trueU = u;
        trueU *= dsVoltage / 12.0;
        trueXhat = frc::RungeKutta(frc3512::DrivetrainController::Dynamics,
                                   trueXhat, trueU, dt);
    }
    EXPECT_TRUE(controller.AtGoal());
}
