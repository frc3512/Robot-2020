// Copyright (c) FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <wpi/numbers>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoShootNine() {
    // Initial Pose - Right in line with the three balls in the Trench Run
    const frc::Pose2d kInitialPose{12_m, 1.05_m,
                                   units::radian_t{wpi::numbers::pi}};
    // Mid Pose - Drive forward slightly
    const frc::Pose2d kEndTrenchShoot{12_m - 1.5 * Drivetrain::kLength, 1.05_m,
                                      units::radian_t{wpi::numbers::pi}};
    // End Pose - Third/Farthest ball in the Trench Run
    const frc::Pose2d kLastTrenchBall{7.95_m, 1.05_m,
                                      units::radian_t{wpi::numbers::pi}};
    const frc::Pose2d kShootToGenPose{
        9.5_m, 2.6_m, units::radian_t{(2 * wpi::numbers::pi) / 3}};

    const frc::Pose2d kGenPose{8.5_m, 4.2_m,
                               units::radian_t{(wpi::numbers::pi * 2) / 3}};

    drivetrain.Reset(kInitialPose);

    // Move back to shoot three comfortably
    drivetrain.AddTrajectory(kInitialPose, {}, kEndTrenchShoot);

    intake.Deploy();

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    if constexpr (IsSimulation()) {
        for (int i = 0; i < 3; ++i) {
            intakeSim.AddBall();
        }
    }
    ShootWithVision(3);

    if (!m_autonChooser.Suspend([=] { return !IsShooting(); })) {
        return;
    }

    // Add a constraint to slow down the drivetrain while it's
    // approaching the balls
    frc::RectangularRegionConstraint firstRegionConstraint{
        frc::Translation2d{kLastTrenchBall.X(),
                           kLastTrenchBall.Y() - 0.5 * Drivetrain::kLength},
        // X: First/Closest ball in the trench run
        frc::Translation2d{9.82_m + 0.5 * Drivetrain::kLength,
                           kInitialPose.Y() + 0.5 * Drivetrain::kLength},
        frc::MaxVelocityConstraint{1.6_mps}};

    frc::RectangularRegionConstraint secondRegionConstraint{
        frc::Translation2d{kGenPose.X() - 0.5 * Drivetrain::kLength, 3_m},
        frc::Translation2d{kGenPose.X() + 0.5 * Drivetrain::kLength,
                           kGenPose.Y() + 0.5 * Drivetrain::kLength},
        frc::MaxVelocityConstraint{1.6_mps}};

    {
        auto config = Drivetrain::MakeTrajectoryConfig();
        config.AddConstraint(firstRegionConstraint);
        drivetrain.AddTrajectory(kEndTrenchShoot, {}, kLastTrenchBall, config);
    }

    // Intake Balls x3
    intake.Start();

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    // Drive back
    {
        auto config = Drivetrain::MakeTrajectoryConfig();
        config.AddConstraint(firstRegionConstraint);
        config.SetReversed(true);
        drivetrain.AddTrajectory({kLastTrenchBall, kEndTrenchShoot}, config);
    }

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    if constexpr (IsSimulation()) {
        for (int i = 0; i < 3; ++i) {
            intakeSim.AddBall();
        }
    }
    ShootWithVision(3);

    if (!m_autonChooser.Suspend([=] { return !IsShooting(); })) {
        return;
    }

    {
        auto config = Drivetrain::MakeTrajectoryConfig(
            0_mps, DrivetrainTrajectoryController::kMaxV);
        drivetrain.AddTrajectory(kEndTrenchShoot, {}, kShootToGenPose, config);
    }

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    {
        auto config = Drivetrain::MakeTrajectoryConfig(
            DrivetrainTrajectoryController::kMaxV, 0_mps);
        config.AddConstraint(secondRegionConstraint);
        drivetrain.AddTrajectory({kShootToGenPose, kGenPose}, config);
    }

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    {
        auto config = Drivetrain::MakeTrajectoryConfig();
        config.SetReversed(true);
        drivetrain.AddTrajectory(kGenPose, {}, kShootToGenPose, config);
    }

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    turret.SetGoal(units::radian_t{wpi::numbers::pi / 3}, 0_rad_per_s);
    if (!m_autonChooser.Suspend([=] { return turret.AtGoal(); })) {
        return;
    }

    ShootWithVision(3);

    if (!m_autonChooser.Suspend([=] { return !IsShooting(); })) {
        return;
    }

    intake.Stop();
}

}  // namespace frc3512
