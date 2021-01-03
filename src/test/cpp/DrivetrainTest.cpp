// Copyright (c) 2019-2021 FRC Team 3512. All Rights Reserved.

#include <frc/Notifier.h>
#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <gtest/gtest.h>
#include <wpi/math>

#include "Constants.hpp"
#include "SimulatorTest.hpp"
#include "subsystems/Drivetrain.hpp"

class DrivetrainTest : public frc3512::SimulatorTest {};

TEST_F(DrivetrainTest, ReachesReferenceStraight) {
    frc3512::Drivetrain drivetrain;

    frc3512::SubsystemBase::RunAllAutonomousInit();
    frc::Notifier controllerPeriodic{[&] {
        drivetrain.AutonomousPeriodic();
        drivetrain.ControllerPeriodic();
    }};
    controllerPeriodic.StartPeriodic(frc3512::Constants::kDt);

    frc::Pose2d initialPose{12.65_m, 5.800_m - 0.343_m,
                            units::radian_t{wpi::math::pi}};

    drivetrain.Reset(initialPose);
    drivetrain.AddTrajectory(
        initialPose, {},
        frc::Pose2d(12.65_m - 0.9398_m - 0.5_m, 5.800_m - 0.343_m,
                    units::radian_t{wpi::math::pi}));

    frc::sim::StepTiming(10_s);

    frc3512::SubsystemBase::RunAllDisabledInit();

    EXPECT_TRUE(drivetrain.AtGoal());
}

TEST_F(DrivetrainTest, ReachesReferenceCurve) {
    frc3512::Drivetrain drivetrain;

    frc3512::SubsystemBase::RunAllAutonomousInit();
    frc::Notifier controllerPeriodic{[&] { drivetrain.ControllerPeriodic(); }};
    controllerPeriodic.StartPeriodic(frc3512::Constants::kDt);

    frc::Pose2d initialPose{0_m, 0_m, 0_rad};

    drivetrain.Reset(initialPose);
    drivetrain.AddTrajectory(initialPose, {},
                             frc::Pose2d(4.8768_m, 2.7432_m, 0_rad));

    frc::sim::StepTiming(10_s);

    frc3512::SubsystemBase::RunAllDisabledInit();

    EXPECT_TRUE(drivetrain.AtGoal());
}

TEST_F(DrivetrainTest, ReachesReferenceOffsetCurve) {
    frc3512::Drivetrain drivetrain;

    frc3512::SubsystemBase::RunAllAutonomousInit();
    frc::Notifier controllerPeriodic{[&] { drivetrain.ControllerPeriodic(); }};
    controllerPeriodic.StartPeriodic(frc3512::Constants::kDt);

    frc::Pose2d initialPose{5_m, 2_m, 0_rad};

    drivetrain.Reset(initialPose);
    drivetrain.AddTrajectory(initialPose, {},
                             frc::Pose2d(9.8768_m, 4.7432_m, 0_rad));

    frc::sim::StepTiming(10_s);

    frc3512::SubsystemBase::RunAllDisabledInit();

    EXPECT_TRUE(drivetrain.AtGoal());
}

TEST_F(DrivetrainTest, TrajectoryQueue) {
    frc3512::Drivetrain drivetrain;

    frc3512::SubsystemBase::RunAllAutonomousInit();
    frc::Notifier controllerPeriodic{[&] { drivetrain.ControllerPeriodic(); }};
    controllerPeriodic.StartPeriodic(frc3512::Constants::kDt);

    // Initial Pose - Right in line with the Target Zone
    const frc::Pose2d initialPose{12.89_m, 2.41_m,
                                  units::radian_t{wpi::math::pi}};
    // Mid Pose - Right before first/closest ball in the Trench Run
    const frc::Pose2d midPose{9.82_m + 0.5 * frc3512::Drivetrain::kLength,
                              0.705_m, units::radian_t{wpi::math::pi}};
    // End Pose - Third/Farthest ball in the Trench Run
    const frc::Pose2d endPose{8_m, 0.71_m, units::radian_t{wpi::math::pi}};

    drivetrain.Reset(initialPose);

    frc::RectangularRegionConstraint regionConstraint{
        // X: Leftmost ball on trench run
        frc::Translation2d{endPose.X(),
                           0.71_m - 0.5 * frc3512::Drivetrain::kLength},
        // X: Rightmost ball on trench run
        frc::Translation2d{9.82_m + 0.5 * frc3512::Drivetrain::kLength,
                           0.71_m + 0.5 * frc3512::Drivetrain::kLength},
        frc::MaxVelocityConstraint{1.6_mps}};

    // Add a constraint to slow down the drivetrain while it's
    // approaching the balls. Interior translation is first/closest ball in
    // trench run.
    auto config = frc3512::Drivetrain::MakeTrajectoryConfig();
    config.AddConstraint(regionConstraint);
    drivetrain.AddTrajectory({initialPose, midPose, endPose}, config);

    // Drive back
    auto config2 = frc3512::Drivetrain::MakeTrajectoryConfig();
    config2.SetReversed(true);
    drivetrain.AddTrajectory(endPose, {}, midPose, config2);

    frc::sim::StepTiming(10_s);

    frc3512::SubsystemBase::RunAllDisabledInit();

    EXPECT_TRUE(drivetrain.AtGoal());
}
