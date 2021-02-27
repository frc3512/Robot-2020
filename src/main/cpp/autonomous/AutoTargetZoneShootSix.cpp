// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoTargetZoneShootSix() {
    // Initial Pose - Right in line with the Target Zone
    const frc::Pose2d kInitialPose{12.89_m, 2.41_m,
                                   units::radian_t{wpi::math::pi}};
    // Mid Pose - Right before first/closest ball in the Trench Run
    const frc::Pose2d kMidPose{9.82_m + 0.5 * Drivetrain::kLength, 0.705_m,
                               units::radian_t{wpi::math::pi}};
    // End Pose - Third/Farthest ball in the Trench Run
    const frc::Pose2d kEndPose{8_m, 0.71_m, units::radian_t{wpi::math::pi}};

    drivetrain.Reset(kInitialPose);

    intake.Deploy();

    if constexpr (IsSimulation()) {
        for (int i = 0; i < 3; ++i) {
            intakeSim.AddBall();
        }
    }
    Shoot(3);

    while (IsShooting()) {
        if (!m_autonChooser.Suspend()) {
            return;
        }
    }

    frc::RectangularRegionConstraint regionConstraint{
        // X: Leftmost ball on trench run
        frc::Translation2d{kEndPose.X(), 0.71_m - 0.5 * Drivetrain::kLength},
        // X: Rightmost ball on trench run
        frc::Translation2d{9.82_m + 0.5 * Drivetrain::kLength,
                           0.71_m + 0.5 * Drivetrain::kLength},
        frc::MaxVelocityConstraint{1.6_mps}};

    // Add a constraint to slow down the drivetrain while it's
    // approaching the balls. Interior translation is first/closest ball in
    // trench run.
    {
        auto config = Drivetrain::MakeTrajectoryConfig();
        config.AddConstraint(regionConstraint);
        drivetrain.AddTrajectory({kInitialPose, kMidPose, kEndPose}, config);
    }

    // Intake Balls x3
    intake.Start();

    // Drive back
    {
        auto config = Drivetrain::MakeTrajectoryConfig();
        config.SetReversed(true);
        drivetrain.AddTrajectory(kEndPose, {}, kMidPose, config);
    }

    while (!drivetrain.AtGoal()) {
        if (drivetrain.GetReferencePose().Translation().Distance(
                kEndPose.Translation()) < 1_cm) {
            intake.Stop();
        }

        if (!m_autonChooser.Suspend()) {
            return;
        }
    }

    if constexpr (IsSimulation()) {
        for (int i = 0; i < 3; ++i) {
            intakeSim.AddBall();
        }
    }
    Shoot(3);

    while (IsShooting()) {
        if (!m_autonChooser.Suspend()) {
            return;
        }
    }
}

}  // namespace frc3512
