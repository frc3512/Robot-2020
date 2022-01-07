// Copyright (c) FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <wpi/numbers>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoLeftSideIntake() {
    // Inital Pose - On initiation line between two balls next to color wheel
    const frc::Pose2d kInitialPose{12_m, 7.1_m,
                                   units::radian_t{wpi::numbers::pi}};
    // End Pose - Right before the two balls on the color wheel so intake
    // doesn't hit it
    const frc::Pose2d kEndPose{10_m + Drivetrain::kMiddleOfRobotToIntake, 7.1_m,
                               units::radian_t{wpi::numbers::pi}};

    drivetrain.Reset(kInitialPose);

    // Add a region constraint to slow down the drivetrain while
    // it's approaching the balls
    frc::RectangularRegionConstraint regionConstraint{
        frc::Translation2d{kEndPose.X(),
                           kEndPose.Y() - 0.5 * Drivetrain::kLength},
        frc::Translation2d{kEndPose.X() + 0.5 * Drivetrain::kLength,
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
