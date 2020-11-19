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

    // Target is pi/4 radians CCW if robot is facing at 0 radians with the
    // following target and turret locations
    EXPECT_EQ(controller.CalculateHeading({4_m, 2_m}, {2_m, 0_m}),
              units::radian_t{wpi::math::pi / 4.0});
}

TEST(TurretControllerTest, CalculateAngularVelocity) {
    frc3512::TurretController controller;
    controller.Reset(0_rad);
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

void VerifyHeadingAdjustment(units::meters_per_second_t drivetrainSpeed,
                             units::radian_t drivetrainHeading,
                             frc::Translation2d turretOffsetFromTarget) {
    constexpr auto kBallSpeed = 15_mps;

    const frc::Translation2d kTargetPosition{TargetModel::kCenter.X(),
                                             TargetModel::kCenter.Y()};
    auto turretPosition = kTargetPosition - turretOffsetFromTarget;

    frc::Velocity2d drivetrainVelocity{drivetrainSpeed,
                                       frc::Rotation2d{drivetrainHeading}};

    auto flywheelAngularSpeed = kBallSpeed / 4_in * 2.0 * 1_rad;
    auto ballHeading =
        units::math::atan2(kTargetPosition.Y() - turretPosition.Y(),
                           kTargetPosition.X() - turretPosition.X()) +
        frc3512::TurretController::CalculateHeadingAdjustment(
            turretPosition, drivetrainVelocity, flywheelAngularSpeed);

    frc::Velocity2d ballVelocity{kBallSpeed, frc::Rotation2d{ballHeading}};

    // Verify drivetrain velocity vector + ball velocity vector intersects
    // target
    auto sum = drivetrainVelocity + ballVelocity;
    auto targetDisplacement = kTargetPosition - turretPosition;
    EXPECT_EQ(
        units::math::atan2(sum.Y(), sum.X()),
        units::math::atan2(targetDisplacement.Y(), targetDisplacement.X()));
}

TEST(TurretControllerTest, CalculateHeadingAdjustment) {
    VerifyHeadingAdjustment(3.5_mps, units::radian_t{wpi::math::pi / 2.0},
                            {-2_m, 0_m});
    VerifyHeadingAdjustment(3.5_mps, units::radian_t{wpi::math::pi},
                            {-2_m, 1_m});
}

TEST(TurretControllerTest, ProperDistanceFromTarget) {
    constexpr units::meter_t kDrivetrainX = 12.65_m - 0.9398_m;
    constexpr units::meter_t kDrivetrainY = 2.600_m - 0.343_m;

    {
        frc3512::TurretController controller;
        controller.Enable();
        frc::sim::StepTiming(frc3512::Constants::kDt);

        Eigen::Matrix<double, 7, 1> drivetrainXhat;
        drivetrainXhat << kDrivetrainX.to<double>(), kDrivetrainY.to<double>(),
            wpi::math::pi, 0, 0, 0, 0;

        controller.SetDrivetrainStates(drivetrainXhat);
        Eigen::Matrix<double, 1, 1> y;
        y << 0.0;
        controller.UpdateAndLog(y);
        auto turretPose = controller.DrivetrainToTurretInGlobal(frc::Pose2d{
            kDrivetrainX, kDrivetrainY, units::radian_t{wpi::math::pi}});

        const frc::Pose2d kTargetPoseInGlobal{TargetModel::kCenter.X(),
                                              TargetModel::kCenter.Y(),
                                              units::radian_t{wpi::math::pi}};

        auto distance = turretPose.Translation().Distance(
            kTargetPoseInGlobal.Translation());

        EXPECT_NEAR_UNITS(
            distance,
            629.25_in - kDrivetrainX +
                frc3512::TurretController::kDrivetrainToTurretFrame.X(),
            1e-6_m);
    }

    frc3512::AddPrefixToCSVs("TurretControllerTest ProperDistanceFromTarget");
}
