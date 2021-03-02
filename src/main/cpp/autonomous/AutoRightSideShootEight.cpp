// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoRightSideShootEight() {
    // Initial Pose - Right in line with the three balls in the Trench Run
    const frc::Pose2d kInitialPose{12.89_m, 0.71_m,
                                   units::radian_t{wpi::math::pi}};
    // Mid Pose - Third/Farthest ball in the Trench Run
    const frc::Pose2d kTrenchPose{8_m, 0.71_m, units::radian_t{wpi::math::pi}};
    // End Pose - Middle of two balls on right side of generator
    const frc::Pose2d kGenPose{10.3_m, 2.55_m, 115_deg};

    const units::meters_per_second_t kMaxV = 1.6_mps;

    m_drivetrain.Reset(kInitialPose);

    m_intake.Deploy();

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
        m_drivetrain.AddTrajectory(kInitialPose, {}, kTrenchPose, config);
    }

    // Intake Balls x3
    m_intake.Start();

    while (!m_drivetrain.AtGoal()) {
        if (!m_autonChooser.Suspend()) {
            return;
        }
    }

    // Drive back
    {
        auto config = Drivetrain::MakeTrajectoryConfig();
        config.SetReversed(true);
        m_drivetrain.AddTrajectory(kTrenchPose, {}, kInitialPose, config);
    }

    while (!m_drivetrain.AtGoal()) {
        if (!m_autonChooser.Suspend()) {
            return;
        }
    }

    // Intake Balls x2
    {
        frc::RectangularRegionConstraint regionConstraint{
            frc::Translation2d{kGenPose.X() - 0.5 * Drivetrain::kLength,
                               kGenPose.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kGenPose.X() + 0.5 * Drivetrain::kLength,
                               kGenPose.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{kMaxV}};
        auto config = m_drivetrain.MakeTrajectoryConfig();
        config.AddConstraint(regionConstraint);
        m_drivetrain.AddTrajectory(kInitialPose, {}, kGenPose, config);
    }

    while (!m_drivetrain.AtGoal()) {
        if (!m_autonChooser.Suspend()) {
            return;
        }
    }

    if constexpr (IsSimulation()) {
        for (int i = 0; i < 5; ++i) {
            intakeSim.AddBall();
        }
    }

    Shoot(5);

    while (IsShooting()) {
        if (!m_autonChooser.Suspend()) {
            return;
        }
    }

    m_intake.Stop();
}

}  // namespace frc3512
