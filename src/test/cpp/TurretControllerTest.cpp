// Copyright (c) 2019-2020 FRC Team 3512. All Rights Reserved.

#include <string_view>

#include <frc/system/plant/DCMotor.h>
#include <gtest/gtest.h>
#include <simulation/RoboRioSim.h>
#include <units/units.h>
#include <wpi/math>

#include "Constants.hpp"
#include "RenameCSVs.hpp"
#include "TargetModel.hpp"
#include "controllers/DrivetrainController.hpp"
#include "controllers/TurretController.hpp"

#define EXPECT_NEAR_UNITS(val1, val2, eps) \
    EXPECT_LE(units::math::abs((val1) - (val2)), eps)

static constexpr bool kIdealModel = true;

TEST(TurretControllerTest, CalculateHeading) {
    frc3512::TurretController controller;
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

void RunSimulation(
    frc3512::DrivetrainController& drivetrainController,
    frc3512::TurretController& turretController,
    Eigen::Matrix<double, 10, 1> drivetrainX =
        Eigen::Matrix<double, 10, 1>::Zero(),
    Eigen::Matrix<double, 2, 1> turretX = Eigen::Matrix<double, 2, 1>::Zero()) {
    using frc3512::Constants::kDt;

    Eigen::Matrix<double, 2, 1> drivetrainU =
        Eigen::Matrix<double, 2, 1>::Zero();
    Eigen::Matrix<double, 1, 1> turretU = Eigen::Matrix<double, 1, 1>::Zero();

    frc::sim::RoboRioSim roboRIO{0};

    auto currentTime = 0_s;
    while (currentTime < 10_s) {
        auto dt = kDt;
        if constexpr (!kIdealModel) {
            dt += units::second_t{frc::MakeWhiteNoiseVector(0.001)(0)};
        }

        // Update drivetrain controller
        Eigen::Matrix<double, 3, 1> drivetrainY =
            frc3512::DrivetrainController::LocalMeasurementModel(
                drivetrainX, Eigen::Matrix<double, 2, 1>::Zero());
        // Add measurement noise to drivetrain controller
        if constexpr (!kIdealModel) {
            drivetrainY += frc::MakeWhiteNoiseVector(0.0001, 0.005, 0.005);
        }
        drivetrainController.SetMeasuredLocalOutputs(
            units::radian_t{drivetrainY(0)}, units::meter_t{drivetrainY(1)},
            units::meter_t{drivetrainY(2)});
        drivetrainController.Update(kDt, currentTime);

        // Update turret controller
        turretController.SetDrivetrainStatus(drivetrainX);
        turretController.SetMeasuredOutputs(units::radian_t{turretX(0)});
        turretController.Update(kDt, currentTime);

        drivetrainU = drivetrainController.GetInputs();
        turretU = turretController.GetInputs();

        // Account for battery voltage drop due to current draw for the turret
        if constexpr (!kIdealModel) {
            constexpr auto Vbat = 12_V;
            constexpr auto Rbat = 0.03_Ohm;
            constexpr auto r = 0.0746125_m;  // Drivetrain wheel radius

            using Input = frc3512::DrivetrainController::Input;
            using State = frc3512::DrivetrainController::State;
            constexpr auto turretMotor = frc::DCMotor::NEO(1);
            units::ampere_t turretLoad =
                turretMotor.Current(units::radians_per_second_t{turretX(1)},
                                    units::volt_t{turretU(0)});
            constexpr auto drivetrainMotors = frc::DCMotor::NEO(2);
            units::ampere_t loadIleft = drivetrainMotors.Current(
                units::meters_per_second_t{drivetrainX(State::kLeftVelocity)} /
                    r * 1_rad,
                units::volt_t{drivetrainU(Input::kLeftVoltage)});
            units::ampere_t loadIright = drivetrainMotors.Current(
                units::meters_per_second_t{drivetrainX(State::kRightVelocity)} /
                    r * 1_rad,
                units::volt_t{drivetrainU(Input::kRightVoltage)});
            units::volt_t vLoaded =
                Vbat - turretLoad * Rbat - loadIleft * Rbat - loadIright * Rbat;
            double dsVoltage =
                vLoaded.to<double>() + frc::MakeWhiteNoiseVector(0.1)(0);
            roboRIO.SetVInVoltage(dsVoltage);

            drivetrainU *= dsVoltage / 12.0;
            turretU *= dsVoltage / 12.0;
        }

        drivetrainX = frc::RungeKutta(frc3512::DrivetrainController::Dynamics,
                                      drivetrainX, drivetrainU, dt);
        turretX = turretController.GetPlant().CalculateX(turretX, turretU, dt);
        currentTime += dt;
    }
}

TEST(TurretControllerTest, ReachesReferenceStaticDrivetrain) {
    // Initialize drivetrain controller
    frc3512::DrivetrainController drivetrainController;
    drivetrainController.Enable();

    // Initialize turret controller
    frc3512::TurretController turretController;
    turretController.Enable();

    RunSimulation(drivetrainController, turretController);

    RenameCSVs("TurretControllerTest Static", "./Turret ");

    EXPECT_TRUE(turretController.AtGoal());
}

TEST(TurretControllerTest, DISABLED_ReachesReferenceRotateInPlaceDrivetrain) {
    // TODO: Make the drivetrain actually rotate in place instead of follow an
    // s-curve.

    // Initialize drivetrain controller
    frc3512::DrivetrainController drivetrainController;
    drivetrainController.Enable();
    drivetrainController.SetWaypoints(frc::Pose2d(0_m, 0_m, 0_rad), {},
                                      frc::Pose2d(4.8768_m, 2.7432_m, 0_rad));

    // Initialize turret controller
    frc3512::TurretController turretController;
    turretController.Enable();

    RunSimulation(drivetrainController, turretController);

    RenameCSVs("TurretController RotateInPlace", "./Drivetrain ");
    RenameCSVs("TurretController RotateInPlace", "./Turret ");

    EXPECT_TRUE(turretController.AtGoal());
}

TEST(TurretControllerTest, ReachesReferenceSCurveDrivetrain) {
    // Initialize drivetrain controller
    frc3512::DrivetrainController drivetrainController;
    drivetrainController.Enable();
    drivetrainController.SetWaypoints(frc::Pose2d(0_m, 0_m, 0_rad), {},
                                      frc::Pose2d(4.8768_m, 2.7432_m, 0_rad));

    // Initialize turret controller
    frc3512::TurretController turretController;
    turretController.Enable();

    RunSimulation(drivetrainController, turretController);

    RenameCSVs("TurretControllerTest SCurve", "./Drivetrain ");
    RenameCSVs("TurretControllerTest SCurve", "./Turret ");

    EXPECT_TRUE(turretController.AtGoal());
}

TEST(TurretControllerTest, ReachesReferenceAutonDrivetrain) {
    frc::Pose2d initialPose{12.65_m, 5.800_m, units::radian_t{wpi::math::pi}};

    // Initialize drivetrain controller
    frc3512::DrivetrainController drivetrainController;
    drivetrainController.Reset(initialPose, initialPose);
    drivetrainController.Enable();
    drivetrainController.SetWaypoints(
        initialPose, {},
        frc::Pose2d(12.65_m - frc3512::DrivetrainController::kLength, 5.800_m,
                    units::radian_t{wpi::math::pi}));

    // Initialize turret controller
    frc3512::TurretController turretController;
    turretController.Enable();

    Eigen::Matrix<double, 10, 1> drivetrainX =
        Eigen::Matrix<double, 10, 1>::Zero();
    drivetrainX(2) = wpi::math::pi;

    RunSimulation(drivetrainController, turretController, drivetrainX);

    RenameCSVs("TurretControllerTest Auton", "./Drivetrain ");
    RenameCSVs("TurretControllerTest Auton", "./Turret ");

    EXPECT_TRUE(turretController.AtGoal());
}

TEST(TurretControllerTest, ProperDistanceFromTarget) {
    constexpr units::meter_t kDrivetrainX = 12.65_m - 0.9398_m;
    constexpr units::meter_t kDrivetrainY = 2.600_m - 0.343_m;

    frc3512::TurretController controller;
    controller.Enable();

    Eigen::Matrix<double, 10, 1> drivetrainXhat;
    drivetrainXhat << kDrivetrainX.to<double>(), kDrivetrainY.to<double>(),
        wpi::math::pi, 0, 0, 0, 0, 0, 0, 0;

    controller.SetDrivetrainStatus(drivetrainXhat);
    controller.Update(0.00505_s, 0_s);
    auto turretPose = controller.GetNextPose();

    const frc::Pose2d kTargetPoseInGlobal{TargetModel::kCenter.X(),
                                          TargetModel::kCenter.Y(),
                                          units::radian_t{wpi::math::pi}};

    auto distance =
        turretPose.Translation().Distance(kTargetPoseInGlobal.Translation());

    RenameCSVs("TurretControllerTest ProperDistanceFromTarget", "./Turret ");

    EXPECT_NEAR_UNITS(
        distance,
        629.25_in - kDrivetrainX +
            frc3512::TurretController::kDrivetrainToTurretFrame.Translation()
                .X(),
        1e-6_m);
}
