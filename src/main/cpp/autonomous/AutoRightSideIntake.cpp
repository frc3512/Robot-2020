// Copyright (c) FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <wpi/numbers>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoRightSideIntake() {
    // Initial Pose - Right in line with the three balls in the Trench Run
    const frc::Pose2d kInitialPose{12_m, 1.05_m,
                                   units::radian_t{wpi::numbers::pi}};
    // End Pose - Second ball in the Trench Run
    const frc::Pose2d kEndPose{7.95_m, 1.05_m,
                               units::radian_t{wpi::numbers::pi}};

    drivetrain.Reset(kInitialPose);

    // Add a constraint to slow down the drivetrain while it's
    // approaching the balls
    frc::RectangularRegionConstraint regionConstraint{
        frc::Translation2d{kEndPose.X(),
                           kEndPose.Y() - 0.5 * Drivetrain::kLength},
        // X: First/Closest ball in the trench run
        frc::Translation2d{9.82_m + 0.5 * Drivetrain::kLength,
                           kInitialPose.Y() + 0.5 * Drivetrain::kLength},
        frc::MaxVelocityConstraint{1_mps}};

    auto config = Drivetrain::MakeTrajectoryConfig();
    config.AddConstraint(regionConstraint);
    drivetrain.AddTrajectory(kInitialPose, {}, kEndPose, config);

    // Intake Balls x2
    intake.Deploy();
    intake.Start();

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    intake.Stop();

    EXPECT_TRUE(turret.AtGoal());
}

}  // namespace frc3512
