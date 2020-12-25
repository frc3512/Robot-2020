// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoRightSideIntake() {
    // Initial Pose - Right in line with the three balls in the Trench Run
    const frc::Pose2d initialPose{12.89_m, 0.71_m,
                                  units::radian_t{wpi::math::pi}};
    // End Pose - Second ball in the Trench Run
    const frc::Pose2d endPose{8.906_m, 0.71_m, units::radian_t{wpi::math::pi}};

    m_drivetrain.Reset(initialPose);

    // Add a constraint to slow down the drivetrain while it's
    // approaching the balls
    frc::RectangularRegionConstraint regionConstraint{
        frc::Translation2d{endPose.X(),
                           endPose.Y() - 0.5 * Drivetrain::kLength},
        // X: First/Closest ball in the trench run
        frc::Translation2d{9.82_m + 0.5 * Drivetrain::kLength,
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
