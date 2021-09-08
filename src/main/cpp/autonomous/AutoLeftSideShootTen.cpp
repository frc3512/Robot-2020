// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <wpi/numbers>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoLeftSideShootTen() {
    // Inital Pose - On initiation line between two balls next to color wheel
    const frc::Pose2d kInitialPose{12.89_m, 7.513_m,
                                   units::radian_t{wpi::numbers::pi}};
    // Left Pickup Pose - Right before the two balls on the color wheel so
    // intake doesn't hit it
    const frc::Pose2d kLeftPickupPose{
        9.63_m + Drivetrain::kMiddleOfRobotToIntake, 7.513_m,
        units::radian_t{wpi::numbers::pi}};
    // Firing Zone Pose! - Drive towards the target to be directly in front of
    // it
    const frc::Pose2d kFiringZonePose{11.89_m, 2.41_m,
                                      units::radian_t{wpi::numbers::pi / 2}};
    // Pre Gen Pose - Helps robot not get a kink while turning to the generator
    const frc::Pose2d kPreGenPose{11.89_m, 1_m,
                                  units::radian_t{wpi::numbers::pi / 2}};
    // Gen Pose - Middle of two balls on right side of generator
    const frc::Pose2d kGenPose{10.2_m, 2.386_m, 115_deg};
    // Front Trench Run Pose - Right before the start of trench run
    const frc::Pose2d kFrontTrenchRunPose{9.82_m + 0.5 * Drivetrain::kLength,
                                          0.705_m,
                                          units::radian_t{wpi::numbers::pi}};
    // Last Ball Pose - Third/Farthest ball in the Trench Run
    const frc::Pose2d kLastBallPose{8_m, 0.71_m,
                                    units::radian_t{wpi::numbers::pi}};
    const frc::Pose2d kFinalShootPose{11.89_m, 2.41_m,
                                      units::radian_t{wpi::numbers::pi}};

    constexpr units::meters_per_second_t kMaxV = 1.6_mps;

    auto forwardConfig = Drivetrain::MakeTrajectoryConfig();
    auto reverseConfig = Drivetrain::MakeTrajectoryConfig();
    reverseConfig.SetReversed(true);

    drivetrain.Reset(kInitialPose);

    intake.Deploy();

    {
        // Left Pickup Region Constraint
        frc::RectangularRegionConstraint regionConstraint{
            frc::Translation2d{kLeftPickupPose.X(),
                               kLeftPickupPose.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kLeftPickupPose.X() + 0.5 * Drivetrain::kLength,
                               kInitialPose.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{kMaxV}};
        auto config = drivetrain.MakeTrajectoryConfig();
        config.AddConstraint(regionConstraint);
        drivetrain.AddTrajectory(kInitialPose, {}, kLeftPickupPose, config);
    }

    // Intake Balls x2
    intake.Start();

    if constexpr (IsSimulation()) {
        for (int i = 0; i < 5; ++i) {
            intakeSim.AddBall();
        }
    }

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    intake.Stop();

    // Left back up
    drivetrain.AddTrajectory(kLeftPickupPose, {}, kFiringZonePose,
                             reverseConfig);

    // First fire
    Shoot(5);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    drivetrain.AddTrajectory(kFiringZonePose, {}, kPreGenPose, reverseConfig);

    // Right generator pickup
    {
        // Generator Region Constraint
        frc::RectangularRegionConstraint regionConstraint{
            frc::Translation2d{9.43_m, 2.65_m},
            frc::Translation2d{9.93_m, 3.61_m},
            frc::MaxVelocityConstraint{kMaxV}};
        auto config = drivetrain.MakeTrajectoryConfig();
        config.AddConstraint(regionConstraint);
        drivetrain.AddTrajectory(kPreGenPose, {}, kGenPose);
    }

    // Intake Balls x2
    intake.Start();

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    intake.Stop();

    drivetrain.AddTrajectory(kGenPose, {}, kFrontTrenchRunPose, reverseConfig);

    {
        // Trench Run Region Constraint
        frc::RectangularRegionConstraint regionConstraint{
            frc::Translation2d{kLastBallPose.X(),
                               0.71_m - 0.5 * Drivetrain::kLength},
            frc::Translation2d{9.82_m + 0.5 * Drivetrain::kLength,
                               0.71_m + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{kMaxV}};
        auto config = drivetrain.MakeTrajectoryConfig();
        config.AddConstraint(regionConstraint);
        drivetrain.AddTrajectory(kFrontTrenchRunPose, {}, kLastBallPose,
                                 config);
    }

    // Intake Balls x3
    intake.Start();

    if constexpr (IsSimulation()) {
        for (int i = 0; i < 5; ++i) {
            intakeSim.AddBall();
        }
    }

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    intake.Stop();

    drivetrain.AddTrajectory(kLastBallPose, {}, kFinalShootPose, reverseConfig);

    Shoot(5);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    EXPECT_TRUE(turret.AtGoal());
}

}  // namespace frc3512
