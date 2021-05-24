// Copyright (c) 2019-2021 FRC Team 3512. All Rights Reserved.

#include <cmath>

#include <frc/simulation/SimHooks.h>
#include <gtest/gtest.h>
#include <units/math.h>
#include <wpi/numbers>

#include "RealTimeRobot.hpp"
#include "SimulatorTest.hpp"
#include "TargetModel.hpp"
#include "controllers/TurretController.hpp"

#define EXPECT_NEAR_UNITS(val1, val2, eps) \
    EXPECT_LE(units::math::abs((val1) - (val2)), eps)

class TurretControllerTest : public frc3512::SimulatorTest {};

TEST_F(TurretControllerTest, CalculateHeading) {
    frc3512::TurretController controller;

    // Target is pi/4 radians CCW if robot is facing at 0 radians with the
    // following target and turret locations
    EXPECT_EQ(controller.CalculateHeading({4_m, 2_m}, {2_m, 0_m}),
              units::radian_t{wpi::numbers::pi / 4.0});
}

TEST_F(TurretControllerTest, CalculateAngularVelocity) {
    frc3512::TurretController controller;
    controller.Reset(0_rad);

    frc::Translation2d targetTranslationInGlobal{4_m, 2_m};
    frc::Translation2d turretTranslationInGlobal{2_m, 0_m};

    auto translationFromTargetInGlobal =
        turretTranslationInGlobal - targetTranslationInGlobal;

    // Moving along x axis
    // Component of v perpendicular to target should be 1/std::sqrt(2) * v =
    // std::sqrt(2)
    //
    // Magnitude of translation to target is sqrt(2^2 + 2^2) = 2 sqrt(2)
    //
    // ω = v perp / r to target = std::sqrt(2) / (2 sqrt(2)) = 0.5
    auto omega = controller.CalculateAngularVelocity(
        {2_mps, 0_mps}, translationFromTargetInGlobal);
    EXPECT_EQ(omega, 0.5_rad_per_s);
    omega = controller.CalculateAngularVelocity({-2_mps, 0_mps},
                                                translationFromTargetInGlobal);
    EXPECT_EQ(omega, -0.5_rad_per_s);
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

    // Compare cos and sin components separately to handle wraparound
    frc::Rotation2d sumRotation{sum.X().to<double>(), sum.Y().to<double>()};
    frc::Rotation2d displacementRotation{targetDisplacement.X().to<double>(),
                                         targetDisplacement.Y().to<double>()};
    EXPECT_NEAR(sumRotation.Cos(), displacementRotation.Cos(), 1e-6);
    EXPECT_NEAR(sumRotation.Sin(), displacementRotation.Sin(), 1e-6);
}

TEST_F(TurretControllerTest, CalculateHeadingAdjustment) {
    VerifyHeadingAdjustment(3.5_mps, units::radian_t{wpi::numbers::pi / 2.0},
                            {-2_m, 0_m});
    VerifyHeadingAdjustment(3.5_mps, units::radian_t{wpi::numbers::pi},
                            {-2_m, 1_m});
}

TEST_F(TurretControllerTest, ProperDistanceFromTarget) {
    constexpr units::meter_t kDrivetrainX = 12.65_m - 0.9398_m;
    constexpr units::meter_t kDrivetrainY = 2.600_m - 0.343_m;

    frc3512::TurretController controller;
    frc::sim::StepTiming(frc3512::RealTimeRobot::kDefaultControllerPeriod);

    Eigen::Matrix<double, 7, 1> drivetrainXhat;
    drivetrainXhat << kDrivetrainX.to<double>(), kDrivetrainY.to<double>(),
        wpi::numbers::pi, 0, 0, 0, 0;

    controller.SetDrivetrainStates(drivetrainXhat);
    Eigen::Matrix<double, 1, 1> y;
    y << 0.0;
    auto turretPose = controller.DrivetrainToTurretInGlobal(frc::Pose2d{
        kDrivetrainX, kDrivetrainY, units::radian_t{wpi::numbers::pi}});

    const frc::Pose2d kTargetPoseInGlobal{TargetModel::kCenter.X(),
                                          TargetModel::kCenter.Y(),
                                          units::radian_t{wpi::numbers::pi}};

    auto distance =
        turretPose.Translation().Distance(kTargetPoseInGlobal.Translation());

    EXPECT_NEAR_UNITS(
        distance,
        TargetModel::kCenter.X() - kDrivetrainX +
            frc3512::TurretController::kDrivetrainToTurretFrame.X(),
        1e-6_m);
}
