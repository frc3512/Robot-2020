// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <wpi/numbers>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoLeftSideIntake() {
    // Inital Pose - On initiation line between two balls next to color wheel
    const frc::Pose2d kInitialPose{12.89_m, 7.513_m,
                                   units::radian_t{wpi::numbers::pi}};
    // End Pose - Right before the two balls on the color wheel so intake
    // doesn't hit it
    const frc::Pose2d kEndPose{9.63_m + Drivetrain::kMiddleOfRobotToIntake,
                               7.513_m, units::radian_t{wpi::numbers::pi}};

    m_drivetrain.Reset(kInitialPose);

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
    m_drivetrain.AddTrajectory(kInitialPose, {}, kEndPose, config);

    // Intake Balls x2
    m_intake.Deploy();
    m_intake.Start();

    if (!m_autonChooser.Suspend([=] { return m_drivetrain.AtGoal(); })) {
        return;
    }

    m_intake.Stop();
}

}  // namespace frc3512
