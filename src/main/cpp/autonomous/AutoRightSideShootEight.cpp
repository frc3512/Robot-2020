// Copyright (c) FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <wpi/numbers>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoRightSideShootEight() {
    // Initial Pose - Right in line with the three balls in the Trench Run
    const frc::Pose2d kInitialPose{12_m, 1.05_m,
                                   units::radian_t{wpi::numbers::pi}};
    // Third/Farthest ball in the Trench Run
    const frc::Pose2d kTrenchPose{7.95_m, 1.05_m,
                                  units::radian_t{wpi::numbers::pi}};
    // Enters generator
    const frc::Pose2d kGenPose{9.3_m, 3.5_m, 120_deg};

    // End pose - In line with target
    const frc::Pose2d kEnd{9.8_m, 2.50_m, units::radian_t{wpi::numbers::pi}};

    const units::meters_per_second_t kMaxV = 1.6_mps;

    drivetrain.Reset(kInitialPose);

    intake.Deploy();

    if constexpr (IsSimulation()) {
        for (int i = 0; i < 3; ++i) {
            intakeSim.AddBall();
        }
    }

    // Shoot our 3 preloaded balls
    Shoot(3);

    if (!m_autonChooser.Suspend([=] { return !IsShooting(); })) {
        return;
    }

    {
        frc::RectangularRegionConstraint regionConstraint{
            frc::Translation2d{kTrenchPose.X(),
                               kTrenchPose.Y() - 0.5 * Drivetrain::kLength},
            // X: First/Closest ball in the trench run
            frc::Translation2d{9.82_m + 0.5 * Drivetrain::kLength,
                               kInitialPose.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{kMaxV}};
        auto config = Drivetrain::MakeTrajectoryConfig();
        config.AddConstraint(regionConstraint);
        drivetrain.AddTrajectory(kInitialPose, {}, kTrenchPose, config);
    }

    // Intake Balls x3
    intake.Start();

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    // Drive back
    {
        auto config = Drivetrain::MakeTrajectoryConfig();
        config.SetReversed(true);
        drivetrain.AddTrajectory(kTrenchPose, {}, kInitialPose, config);
    }

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    // Intake Balls x2
    {
        frc::RectangularRegionConstraint regionConstraint{
            frc::Translation2d{kGenPose.X() - 0.5 * Drivetrain::kLength,
                               kGenPose.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kGenPose.X() + 0.5 * Drivetrain::kLength,
                               kGenPose.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{kMaxV}};
        auto config = drivetrain.MakeTrajectoryConfig();
        config.AddConstraint(regionConstraint);
        drivetrain.AddTrajectory({kInitialPose, kGenPose, kEnd}, config);
    }

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    if constexpr (IsSimulation()) {
        for (int i = 0; i < 5; ++i) {
            intakeSim.AddBall();
        }
    }

    Shoot(5);

    if (!m_autonChooser.Suspend([=] { return !IsShooting(); })) {
        return;
    }

    intake.Stop();

    EXPECT_TRUE(turret.AtGoal());
}

}  // namespace frc3512
