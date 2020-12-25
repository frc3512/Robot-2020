// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoLeftSideIntake() {
    // Inital Pose - On initiation line between two balls next to color wheel
    const frc::Pose2d initialPose{12.89_m, 7.513_m,
                                  units::radian_t{wpi::math::pi}};
    // End Pose - Right before the two balls on the color wheel so intake
    // doesn't hit it
    const frc::Pose2d endPose{9.63_m + Drivetrain::kMiddleOfRobotToIntake,
                              7.513_m, units::radian_t{wpi::math::pi}};

    m_drivetrain.Reset(initialPose);

    // Add a region constraint to slow down the drivetrain while
    // it's approaching the balls
    frc::RectangularRegionConstraint regionConstraint{
        frc::Translation2d{endPose.X(),
                           endPose.Y() - 0.5 * Drivetrain::kLength},
        frc::Translation2d{endPose.X() + 0.5 * Drivetrain::kLength,
                           initialPose.Y() + 0.5 * Drivetrain::kLength},
        frc::MaxVelocityConstraint{1_mps}};

    auto config = Drivetrain::MakeTrajectoryConfig();
    config.AddConstraint(regionConstraint);
    m_drivetrain.SetWaypoints(initialPose, {}, endPose, config);

    // Intake Balls x2
    m_intake.Deploy();
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
    m_intake.SetFunnel(0);
}

}  // namespace frc3512
