// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoTargetZoneShootSix() {
    // Initial Pose - Right in line with the Target Zone
    const frc::Pose2d initialPose{12.89_m, 2.41_m,
                                  units::radian_t{wpi::math::pi}};
    // Mid Pose - Right before first/closest ball in the Trench Run
    const frc::Pose2d midPose{9.82_m + 0.5 * Drivetrain::kLength, 0.705_m,
                              units::radian_t{wpi::math::pi}};
    // End Pose - Third/Farthest ball in the Trench Run
    const frc::Pose2d endPose{8_m, 0.71_m, units::radian_t{wpi::math::pi}};

    m_drivetrain.Reset(initialPose);

    m_intake.Deploy();

    // Shoot x3
    Shoot();

    while (IsShooting()) {
        m_autonChooser.YieldToMain();
        if (!IsAutonomousEnabled()) {
            EXPECT_TRUE(false) << "Autonomous mode didn't complete";
            return;
        }
    }

    frc::RectangularRegionConstraint regionConstraint{
        // X: Leftmost ball on trench run
        frc::Translation2d{endPose.X(), 0.71_m - 0.5 * Drivetrain::kLength},
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
        m_drivetrain.AddTrajectory({initialPose, midPose, endPose}, config);
    }

    // Intake Balls x3
    m_intake.SetArmMotor(Intake::ArmMotorDirection::kIntake);
    m_intake.SetFunnel(0.4);

    while (!m_drivetrain.AtGoal()) {
        m_autonChooser.YieldToMain();
        if (!IsAutonomousEnabled()) {
            EXPECT_TRUE(false) << "Autonomous mode didn't complete";
            return;
        }
    }

    m_intake.SetArmMotor(Intake::ArmMotorDirection::kIdle);
    m_intake.SetFunnel(0.0);

    // Drive back
    {
        auto config = Drivetrain::MakeTrajectoryConfig();
        config.SetReversed(true);
        m_drivetrain.AddTrajectory(endPose, {}, midPose, config);
    }

    while (!m_drivetrain.AtGoal()) {
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
