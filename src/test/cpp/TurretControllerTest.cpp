// Copyright (c) 2019-2020 FRC Team 3512. All Rights Reserved.

#include <frc/simulation/SimHooks.h>
#include <gtest/gtest.h>
#include <units/math.h>
#include <wpi/math>

#include "CSVTestUtil.hpp"
#include "Constants.hpp"
#include "TargetModel.hpp"
#include "controllers/TurretController.hpp"

#define EXPECT_NEAR_UNITS(val1, val2, eps) \
    EXPECT_LE(units::math::abs((val1) - (val2)), eps)

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

TEST(TurretControllerTest, CalculateAngularVelocity) {
    frc3512::TurretController controller;
    controller.Reset();
    controller.Enable();

    Eigen::Vector2d targetTranslationInGlobal;
    targetTranslationInGlobal << 4.0, 2.0;

    Eigen::Vector2d turretTranslationInGlobal;
    turretTranslationInGlobal << 2.0, 0.0;

    Eigen::Vector2d translationToTargetInGlobal =
        targetTranslationInGlobal - turretTranslationInGlobal;

    // Moving along x axis
    Eigen::Vector2d turretVelocityInGlobal;
    turretVelocityInGlobal << 2.0, 0.0;

    // Component of v perpendicular to target should be 1/std::sqrt(2) * v =
    // std::sqrt(2)
    //
    // Magnitude of translation to target is sqrt(2^2 + 2^2) = 2 sqrt(2)
    //
    // Omega = v perp / r to target = std::sqrt(2) / (2 sqrt(2)) = 0.5
    auto omega = controller.CalculateAngularVelocity(
        turretVelocityInGlobal, translationToTargetInGlobal);

    EXPECT_EQ(omega, 0.5_rad_per_s);
}

TEST(TurretControllerTest, ProperDistanceFromTarget) {
    constexpr units::meter_t kDrivetrainX = 12.65_m - 0.9398_m;
    constexpr units::meter_t kDrivetrainY = 2.600_m - 0.343_m;

    frc3512::TurretController controller;
    controller.Enable();
    frc::sim::StepTiming(frc3512::Constants::kDt);

    Eigen::Matrix<double, 10, 1> drivetrainXhat;
    drivetrainXhat << kDrivetrainX.to<double>(), kDrivetrainY.to<double>(),
        wpi::math::pi, 0, 0, 0, 0, 0, 0, 0;

    controller.SetDrivetrainStates(drivetrainXhat);
    Eigen::Matrix<double, 1, 1> y;
    y << 0.0;
    controller.UpdateAndLog(y);
    auto turretPose = controller.GetNextPose();

    const frc::Pose2d kTargetPoseInGlobal{TargetModel::kCenter.X(),
                                          TargetModel::kCenter.Y(),
                                          units::radian_t{wpi::math::pi}};

    auto distance =
        turretPose.Translation().Distance(kTargetPoseInGlobal.Translation());

    frc3512::AddPrefixToCSVs("TurretControllerTest ProperDistanceFromTarget");

    EXPECT_NEAR_UNITS(
        distance,
        629.25_in - kDrivetrainX +
            frc3512::TurretController::kDrivetrainToTurretFrame.X(),
        1e-6_m);
}
