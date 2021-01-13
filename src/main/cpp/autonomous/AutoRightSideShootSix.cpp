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

    drivetrain.Reset(kInitialPose);

    // Move back to shoot three comfortably
    drivetrain.AddTrajectory(kInitialPose, {}, kMidPose);

    intake.Deploy();

    while (!drivetrain.AtGoal()) {
        m_autonChooser.YieldToMain();
        if (!IsAutonomousEnabled()) {
            EXPECT_TRUE(false) << "Autonomous mode didn't complete";
            return;
        }
    }

    // Shoot x3
    Shoot();

    while (IsShooting()) {
        m_autonChooser.YieldToMain();
        if (!IsAutonomousEnabled()) {
            EXPECT_TRUE(false) << "Autonomous mode didn't complete";
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
        drivetrain.AddTrajectory(kMidPose, {}, kEndPose, config);
    }

    // Intake Balls x3
    intake.SetArmMotor(Intake::ArmMotorDirection::kIntake);
    intake.SetFunnel(0.4);

    while (!drivetrain.AtGoal()) {
        m_autonChooser.YieldToMain();
        if (!IsAutonomousEnabled()) {
            EXPECT_TRUE(false) << "Autonomous mode didn't complete";
            return;
        }
    }

    intake.SetArmMotor(Intake::ArmMotorDirection::kIdle);
    intake.SetFunnel(0.0);

    // Drive back
    {
        auto config = Drivetrain::MakeTrajectoryConfig();
        config.SetReversed(true);
        drivetrain.AddTrajectory({kEndPose, kMidPose}, config);
    }

    while (!drivetrain.AtGoal()) {
        m_autonChooser.YieldToMain();
        if (!IsAutonomousEnabled()) {
            EXPECT_TRUE(false) << "Autonomous mode didn't complete";
            return;
        }
    }

    // Shoot x3
    Shoot();

    while (IsShooting()) {
        m_autonChooser.YieldToMain();
        if (!IsAutonomousEnabled()) {
            EXPECT_TRUE(false) << "Autonomous mode didn't complete";
            return;
        }
    }
}

}  // namespace frc3512
