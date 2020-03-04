// Copyright (c) 2019-2020 FRC Team 3512. All Rights Reserved.

#include <experimental/filesystem>
#include <string>
#include <string_view>

#include <frc/system/plant/DCMotor.h>
#include <gtest/gtest.h>
#include <mockdata/RoboRioData.h>
#include <units/units.h>
#include <wpi/math>

#include "Constants.hpp"
#include "controllers/DrivetrainController.hpp"
#include "controllers/TurretController.hpp"

static constexpr bool kIdealModel = true;

/**
 * Rename turret CSVs to avoid them being overwritten
 *
 * @param prefix Prefix for turret CSV filename
 */
void RenameTurretCSVs(std::string_view prefix) {
    for (auto& p : std::experimental::filesystem::directory_iterator(".")) {
        std::string directoryEntry = p.path().u8string();
        std::string oldFilename =
            directoryEntry.substr(directoryEntry.find("/") + 1);
        if (directoryEntry.find("./Turret") == 0) {
            std::string newFilename =
                std::string{prefix.data()} + " " + oldFilename;
            std::rename(oldFilename.c_str(), newFilename.c_str());
        }
    }
}

TEST(TurretControllerTest, CalculateHeading) {
    frc3512::TurretController controller;
    controller.Reset();
    controller.Enable();

    Eigen::Vector2d targetTranslationInGlobal;
    targetTranslationInGlobal << 4.0, 2.0;

    Eigen::Vector2d turretTranslationInGlobal;
    turretTranslationInGlobal << 2.0, 0.0;

    auto theta = controller.CalculateHeading(targetTranslationInGlobal,
                                             turretTranslationInGlobal);

    // Target is pi/4 radians CCW if robot is facing at 0 radians with the
    // turret and target locations above
    EXPECT_EQ(theta, units::radian_t{wpi::math::pi / 4.0});
}

TEST(TurretControllerTest, ReachesReferenceStaticDrivetrain) {
    using frc3512::Constants::kDt;

    frc3512::TurretController controller;
    controller.Reset();
    controller.Enable();

    controller.SetMeasuredOutputs(0_rad);
    Eigen::Matrix<double, 10, 1> drivetrainXhat =
        Eigen::Matrix<double, 10, 1>::Zero();
    controller.SetDrivetrainStatus(drivetrainXhat);

    Eigen::Matrix<double, 2, 1> trueXhat = Eigen::Matrix<double, 2, 1>::Zero();

    auto currentTime = 0_s;
    while (currentTime < 10_s) {
        auto dt = kDt;
        if constexpr (!kIdealModel) {
            dt += units::second_t{frc::MakeWhiteNoiseVector(0.001)(0, 0)};
        }

        controller.SetMeasuredOutputs(units::radian_t{trueXhat(0)});

        controller.Update(kDt, currentTime);
        currentTime += dt;

        constexpr auto Vbat = 12_V;
        constexpr auto Rbat = 0.03_Ohm;
        Eigen::Matrix<double, 1, 1> u;
        u << controller.ControllerVoltage().to<double>();

        if constexpr (!kIdealModel) {
            // Account for battery voltage drop due to current draw from both
            // turret and drivetrain
            constexpr auto motor = frc::DCMotor::NEO(1);
            units::ampere_t load =
                motor.Current(units::radians_per_second_t{trueXhat(1)},
                              units::volt_t{u(0, 0)});
            units::volt_t vLoaded = Vbat - load * Rbat;
            double dsVoltage =
                vLoaded.to<double>() + frc::MakeWhiteNoiseVector(0.1)(0, 0);
            HALSIM_SetRoboRioVInVoltage(0, dsVoltage);
            Eigen::Matrix<double, 1, 1> trueU = u;
            trueU *= dsVoltage / 12.0;
        }

        trueXhat << controller.EstimatedAngle().to<double>(),
            controller.EstimatedAngularVelocity().to<double>();
    }

    RenameTurretCSVs("Static");

    EXPECT_TRUE(controller.AtGoal());
}

TEST(TurretControllerTest, ReachesReferenceRotateInPlaceDrivetrain) {
    using frc3512::Constants::kDt;

    // Initialize turret controller
    frc3512::TurretController turretController;
    turretController.Reset();
    turretController.Enable();
    turretController.SetMeasuredOutputs(0_rad);
    Eigen::Matrix<double, 2, 1> turretXhat =
        Eigen::Matrix<double, 2, 1>::Zero();

    // Initialize drivetrain controller
    frc3512::DrivetrainController drivetrainController{
        {0.0625, 0.125, 10.0, 0.95, 0.95}, {12.0, 12.0}, kDt};
    drivetrainController.Reset(frc::Pose2d{0_m, 0_m, 0_rad});
    drivetrainController.Enable();
    drivetrainController.SetMeasuredLocalOutputs(0_rad, 0_m, 0_m);
    drivetrainController.SetWaypoints(
        {frc::Pose2d(0_m, 0_m, 0_rad), frc::Pose2d(4.8768_m, 2.7432_m, 0_rad)});
    Eigen::Matrix<double, 10, 1> drivetrainTrueXhat =
        Eigen::Matrix<double, 10, 1>::Zero();

    auto currentTime = 0_s;
    while (currentTime < 10_s) {
        auto dt = kDt;
        if constexpr (!kIdealModel) {
            dt += units::second_t{frc::MakeWhiteNoiseVector(0.001)(0, 0)};
        }

        // Update turret controller
        turretController.SetDrivetrainStatus(drivetrainTrueXhat);
        turretController.SetMeasuredOutputs(units::radian_t{turretXhat(0)});
        turretController.Update(kDt, currentTime);

        // Update drivetrain controller
        Eigen::Matrix<double, 3, 1> drivetrainY =
            frc3512::DrivetrainController::LocalMeasurementModel(
                drivetrainTrueXhat, Eigen::Matrix<double, 2, 1>::Zero());
        // Add measurement noise to drivetrain controller
        if constexpr (!kIdealModel) {
            drivetrainY += frc::MakeWhiteNoiseVector(0.0001, 0.005, 0.005);
        }
        drivetrainController.SetMeasuredLocalOutputs(
            units::radian_t{drivetrainY(0, 0)},
            units::meter_t{drivetrainY(1, 0)},
            units::meter_t{drivetrainY(2, 0)});
        drivetrainController.Update(kDt, currentTime);

        currentTime += dt;

        constexpr auto Vbat = 12_V;
        constexpr auto Rbat = 0.03_Ohm;
        constexpr auto r = 0.0746125_m;  // Drivetrain wheel radius
        Eigen::Matrix<double, 2, 1> drivetrainU =
            drivetrainController.GetInputs();
        Eigen::Matrix<double, 1, 1> turretU;
        turretU << turretController.ControllerVoltage().to<double>();

        if constexpr (!kIdealModel) {
            // Account for battery voltage drop due to current draw for the
            // turret
            using Input = frc3512::DrivetrainController::Input;
            using State = frc3512::DrivetrainController::State;
            constexpr auto turretMotor = frc::DCMotor::NEO(1);
            units::ampere_t turretLoad =
                turretMotor.Current(units::radians_per_second_t{turretXhat(1)},
                                    units::volt_t{turretU(0, 0)});
            constexpr auto drivetrainMotors = frc::DCMotor::NEO(2);
            units::ampere_t loadIleft = drivetrainMotors.Current(
                units::meters_per_second_t{
                    drivetrainTrueXhat(State::kLeftVelocity, 0)} /
                    r * 1_rad,
                units::volt_t{drivetrainU(Input::kLeftVoltage, 0)});
            units::ampere_t loadIright = drivetrainMotors.Current(
                units::meters_per_second_t{
                    drivetrainTrueXhat(State::kRightVelocity, 0)} /
                    r * 1_rad,
                units::volt_t{drivetrainU(Input::kRightVoltage, 0)});
            units::volt_t vLoaded =
                Vbat - turretLoad * Rbat - loadIleft * Rbat - loadIright * Rbat;
            double dsVoltage =
                vLoaded.to<double>() + frc::MakeWhiteNoiseVector(0.1)(0, 0);
            HALSIM_SetRoboRioVInVoltage(0, dsVoltage);
            drivetrainU *= dsVoltage / 12.0;
            Eigen::Matrix<double, 1, 1> turretTrueU = turretU;
            turretTrueU *= dsVoltage / 12.0;
        }

        drivetrainTrueXhat =
            frc::RungeKutta(frc3512::DrivetrainController::Dynamics,
                            drivetrainTrueXhat, drivetrainU, dt);
        turretXhat << turretController.EstimatedAngle().to<double>(),
            turretController.EstimatedAngularVelocity().to<double>();
    }

    RenameTurretCSVs("RotateInPlace");

    EXPECT_TRUE(turretController.AtGoal());
}

TEST(TurretControllerTest, ReachesReferenceSCurveDrivetrain) {
    using frc3512::Constants::kDt;

    // Initialize turret controller
    frc3512::TurretController turretController;
    turretController.Reset();
    turretController.Enable();
    turretController.SetMeasuredOutputs(0_rad);
    Eigen::Matrix<double, 2, 1> turretXhat =
        Eigen::Matrix<double, 2, 1>::Zero();

    // Initialize drivetrain controller
    frc3512::DrivetrainController drivetrainController{
        {0.0625, 0.125, 10.0, 0.95, 0.95}, {12.0, 12.0}, kDt};
    drivetrainController.Reset(frc::Pose2d{0_m, 0_m, 0_rad});
    drivetrainController.Enable();
    drivetrainController.SetMeasuredLocalOutputs(0_rad, 0_m, 0_m);
    drivetrainController.SetWaypoints(
        {frc::Pose2d(0_m, 0_m, 0_rad), frc::Pose2d(4.8768_m, 2.7432_m, 0_rad)});
    Eigen::Matrix<double, 10, 1> drivetrainTrueXhat =
        Eigen::Matrix<double, 10, 1>::Zero();

    auto currentTime = 0_s;
    while (currentTime < 10_s) {
        auto dt = kDt;
        if constexpr (!kIdealModel) {
            dt += units::second_t{frc::MakeWhiteNoiseVector(0.001)(0, 0)};
        }

        // Update drivetrain controller
        Eigen::Matrix<double, 3, 1> drivetrainY =
            frc3512::DrivetrainController::LocalMeasurementModel(
                drivetrainTrueXhat, Eigen::Matrix<double, 2, 1>::Zero());
        // Add measurement noise to drivetrain controller
        if constexpr (!kIdealModel) {
            drivetrainY += frc::MakeWhiteNoiseVector(0.0001, 0.005, 0.005);
        }
        drivetrainController.SetMeasuredLocalOutputs(
            units::radian_t{drivetrainY(0, 0)},
            units::meter_t{drivetrainY(1, 0)},
            units::meter_t{drivetrainY(2, 0)});
        drivetrainController.Update(kDt, currentTime);

        // Update turret controller
        turretController.SetDrivetrainStatus(drivetrainTrueXhat);
        turretController.SetMeasuredOutputs(units::radian_t{turretXhat(0)});
        turretController.Update(kDt, currentTime);

        currentTime += dt;

        constexpr auto Vbat = 12_V;
        constexpr auto Rbat = 0.03_Ohm;
        constexpr auto r = 0.0746125_m;  // Drivetrain wheel radius
        Eigen::Matrix<double, 2, 1> drivetrainU =
            drivetrainController.GetInputs();
        Eigen::Matrix<double, 1, 1> turretU;
        turretU << turretController.ControllerVoltage().to<double>();

        if constexpr (!kIdealModel) {
            // Account for battery voltage drop due to current draw for the
            // turret
            using Input = frc3512::DrivetrainController::Input;
            using State = frc3512::DrivetrainController::State;
            constexpr auto turretMotor = frc::DCMotor::NEO(1);
            units::ampere_t turretLoad =
                turretMotor.Current(units::radians_per_second_t{turretXhat(1)},
                                    units::volt_t{turretU(0, 0)});
            constexpr auto drivetrainMotors = frc::DCMotor::NEO(2);
            units::ampere_t loadIleft = drivetrainMotors.Current(
                units::meters_per_second_t{
                    drivetrainTrueXhat(State::kLeftVelocity, 0)} /
                    r * 1_rad,
                units::volt_t{drivetrainU(Input::kLeftVoltage, 0)});
            units::ampere_t loadIright = drivetrainMotors.Current(
                units::meters_per_second_t{
                    drivetrainTrueXhat(State::kRightVelocity, 0)} /
                    r * 1_rad,
                units::volt_t{drivetrainU(Input::kRightVoltage, 0)});
            units::volt_t vLoaded =
                Vbat - turretLoad * Rbat - loadIleft * Rbat - loadIright * Rbat;
            double dsVoltage =
                vLoaded.to<double>() + frc::MakeWhiteNoiseVector(0.1)(0, 0);
            HALSIM_SetRoboRioVInVoltage(0, dsVoltage);
            drivetrainU *= dsVoltage / 12.0;
            Eigen::Matrix<double, 1, 1> turretTrueU = turretU;
            turretTrueU *= dsVoltage / 12.0;
        }

        drivetrainTrueXhat =
            frc::RungeKutta(frc3512::DrivetrainController::Dynamics,
                            drivetrainTrueXhat, drivetrainU, dt);
        turretXhat << turretController.EstimatedAngle().to<double>(),
            turretController.EstimatedAngularVelocity().to<double>();
    }

    RenameTurretCSVs("SCurve");

    EXPECT_TRUE(turretController.AtGoal());
}
