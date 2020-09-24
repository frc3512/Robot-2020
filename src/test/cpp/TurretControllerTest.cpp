// Copyright (c) 2019-2020 FRC Team 3512. All Rights Reserved.

#include <cmath>

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

    frc::Translation2d targetTranslationInGlobal{4_m, 2_m};
    frc::Translation2d turretTranslationInGlobal{2_m, 0_m};

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

    frc::Translation2d targetTranslationInGlobal{4_m, 2_m};
    frc::Translation2d turretTranslationInGlobal{2_m, 0_m};

    auto translationToTargetInGlobal =
        targetTranslationInGlobal - turretTranslationInGlobal;

    // Moving along x axis
    frc::Velocity2d turretVelocityInGlobal{2_mps, 0_mps};

    // Component of v perpendicular to target should be 1/std::sqrt(2) * v =
    // std::sqrt(2)
    //
    // Magnitude of translation to target is sqrt(2^2 + 2^2) = 2 sqrt(2)
    //
    // ω = v perp / r to target = std::sqrt(2) / (2 sqrt(2)) = 0.5
    auto omega = controller.CalculateAngularVelocity(
        turretVelocityInGlobal, translationToTargetInGlobal);

    EXPECT_EQ(omega, 0.5_rad_per_s);
}

TEST(TurretControllerTest, CalculateHeadingAdjustment) {
    constexpr auto kDrivetrainSpeed = 3.5_mps;
    constexpr auto kBallSpeed = 15_mps;

    frc::Translation2d targetPosition{TargetModel::kCenter.X(),
                                      TargetModel::kCenter.Y()};
    auto turretPosition = targetPosition - frc::Translation2d{2_m, 0_m};

    constexpr units::radian_t kDrivetrainHeading{wpi::math::pi / 2.0};

    frc::Velocity2d drivetrainVelocity{kDrivetrainSpeed,
                                       frc::Rotation2d{kDrivetrainHeading}};

    auto flywheelAngularSpeed = kBallSpeed / 4_in * 2.0 * 1_rad;
    auto ballHeading =
        units::math::atan2(targetPosition.Y() - turretPosition.Y(),
                           targetPosition.X() - turretPosition.X()) +
        frc3512::TurretController::CalculateHeadingAdjustment(
            turretPosition, drivetrainVelocity, flywheelAngularSpeed);

    frc::Velocity2d ballVelocity{kBallSpeed, frc::Rotation2d{ballHeading}};

    // Verify drivetrain velocity vector + ball velocity vector intersects
    // target
    auto sum = drivetrainVelocity + ballVelocity;
    auto targetDisplacement = targetPosition - turretPosition;
    EXPECT_EQ(
        units::math::atan2(sum.Y(), sum.X()),
        units::math::atan2(targetDisplacement.Y(), targetDisplacement.X()));
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
