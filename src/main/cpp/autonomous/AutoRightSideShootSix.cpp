// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoRightSideShootSix() {
    // Initial Pose - Right in line with the three balls in the Trench Run
    const frc::Pose2d kInitialPose{12.89_m, 0.71_m,
                                   units::radian_t{wpi::math::pi}};
    // Mid Pose - Drive forward slightly
    const frc::Pose2d kMidPose{12.89_m - 1.5 * Drivetrain::kLength, 0.71_m,
                               units::radian_t{wpi::math::pi}};
    // End Pose - Third/Farthest ball in the Trench Run
    const frc::Pose2d kEndPose{8_m, 0.71_m, units::radian_t{wpi::math::pi}};

    m_drivetrain.Reset(kInitialPose);

    // Move back to shoot three comfortably
    m_drivetrain.AddTrajectory(kInitialPose, {}, kMidPose);

    m_intake.Deploy();

    while (!m_drivetrain.AtGoal()) {
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

    // Add a constraint to slow down the drivetrain while it's
    // approaching the balls
    frc::RectangularRegionConstraint regionConstraint{
        frc::Translation2d{kEndPose.X(),
                           kEndPose.Y() - 0.5 * Drivetrain::kLength},
        // X: First/Closest ball in the trench run
        frc::Translation2d{9.82_m + 0.5 * Drivetrain::kLength,
                           kInitialPose.Y() + 0.5 * Drivetrain::kLength},
        frc::MaxVelocityConstraint{1.6_mps}};

    {
        auto config = Drivetrain::MakeTrajectoryConfig();
        config.AddConstraint(regionConstraint);
        m_drivetrain.AddTrajectory(kMidPose, {}, kEndPose, config);
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
        m_drivetrain.AddTrajectory({kEndPose, kMidPose}, config);
    }

    while (!m_drivetrain.AtGoal()) {
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

    m_intake.Stop();
}

}  // namespace frc3512
