// Copyright (c) 2019-2021 FRC Team 3512. All Rights Reserved.

#include <frc/Notifier.h>
#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <gtest/gtest.h>
#include <wpi/math>

#include "Constants.hpp"
#include "SimulatorTest.hpp"
#include "subsystems/Drivetrain.hpp"

class DrivetrainTest : public frc3512::SimulatorTest {
public:
    frc3512::Drivetrain drivetrain;
    frc::Notifier controllerPeriodic{[&] {
        drivetrain.AutonomousPeriodic();
        drivetrain.ControllerPeriodic();
    }};

    DrivetrainTest() {
        frc3512::SubsystemBase::RunAllAutonomousInit();
        controllerPeriodic.StartPeriodic(frc3512::Constants::kDt);
    }
};

TEST_F(DrivetrainTest, ReachesReferenceStraight) {
    const frc::Pose2d kInitialPose{12.65_m, 5.800_m - 0.343_m,
                                   units::radian_t{wpi::math::pi}};

    drivetrain.Reset(kInitialPose);
    drivetrain.AddTrajectory(
        kInitialPose, {},
        frc::Pose2d(12.65_m - 0.9398_m - 0.5_m, 5.800_m - 0.343_m,
                    units::radian_t{wpi::math::pi}));

    frc::sim::StepTiming(10_s);

    EXPECT_TRUE(drivetrain.AtGoal());
}

TEST_F(DrivetrainTest, ReachesReferenceCurve) {
    const frc::Pose2d kInitialPose{0_m, 0_m, 0_rad};

    drivetrain.Reset(kInitialPose);
    drivetrain.AddTrajectory(kInitialPose, {},
                             frc::Pose2d(4.8768_m, 2.7432_m, 0_rad));

    frc::sim::StepTiming(10_s);

    EXPECT_TRUE(drivetrain.AtGoal());
}

TEST_F(DrivetrainTest, ReachesReferenceOffsetCurve) {
    const frc::Pose2d kInitialPose{5_m, 2_m, 0_rad};

    drivetrain.Reset(kInitialPose);
    drivetrain.AddTrajectory(kInitialPose, {},
                             frc::Pose2d(9.8768_m, 4.7432_m, 0_rad));

    frc::sim::StepTiming(10_s);

    EXPECT_TRUE(drivetrain.AtGoal());
}

TEST_F(DrivetrainTest, TrajectoryQueue) {
    // Initial Pose - Right in line with the Target Zone
    const frc::Pose2d kInitialPose{12.89_m, 2.41_m,
                                   units::radian_t{wpi::math::pi}};
    // Mid Pose - Right before first/closest ball in the Trench Run
    const frc::Pose2d kMidPose{9.82_m + 0.5 * frc3512::Drivetrain::kLength,
                               0.705_m, units::radian_t{wpi::math::pi}};
    const frc::Pose2d kRotatePose{9.82_m + 0.5 * frc3512::Drivetrain::kLength,
                                  0.705_m, 0_rad};
    // End Pose - Third/Farthest ball in the Trench Run
    const frc::Pose2d kEndPose{8_m, 0.71_m, units::radian_t{wpi::math::pi}};

    drivetrain.Reset(kInitialPose);

    frc::RectangularRegionConstraint regionConstraint{
        // X: Leftmost ball on trench run
        frc::Translation2d{kEndPose.X(),
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
    drivetrain.AddTrajectory(
        {kInitialPose, kMidPose, kRotatePose, kMidPose, kEndPose}, config);

    // Drive back
    auto config2 = frc3512::Drivetrain::MakeTrajectoryConfig();
    config2.SetReversed(true);
    drivetrain.AddTrajectory(kEndPose, {}, kMidPose, config2);

    frc::sim::StepTiming(10_s);

    EXPECT_TRUE(drivetrain.AtGoal());
}

TEST_F(DrivetrainTest, CorrectsTowardGlobalY) {
    static constexpr bool kIdealModel = false;

    drivetrain.AddTrajectory(frc::Pose2d(0_m, 0_m, 0_rad), {},
                             frc::Pose2d(4.8768_m, 2.7432_m, 0_rad));

    // Ensure error covariance is nonzero
    frc::sim::StepTiming(10_s);

    Eigen::Matrix<double, 7, 1> x;
    x << 5.0, 5.0, 1.0, 1.0, 1.0, 1.0, 1.0;

    Eigen::Matrix<double, 5, 1> localY =
        frc3512::DrivetrainController::LocalMeasurementModel(
            x, Eigen::Matrix<double, 2, 1>::Zero());
    Eigen::Matrix<double, 2, 1> globalY =
        frc3512::DrivetrainController::GlobalMeasurementModel(
            x, Eigen::Matrix<double, 2, 1>::Zero());
    auto globalTimestamp = frc2::Timer::GetFPGATimestamp();

    // Add measurement noise
    if constexpr (!kIdealModel) {
        localY += frc::MakeWhiteNoiseVector(0.0001, 0.005, 0.005, 7.0, 7.0);
        globalY += frc::MakeWhiteNoiseVector(0.05, 0.05);
    }

    using State = frc3512::DrivetrainController::State;
    using GlobalOutput = frc3512::DrivetrainController::GlobalOutput;

    auto xHat = drivetrain.GetStates();
    double xDiff1 = std::abs(xHat(State::kX) - x(State::kX));
    double yDiff1 = std::abs(xHat(State::kY) - x(State::kY));

    frc::sim::StepTiming(2_s);

    drivetrain.CorrectWithGlobalOutputs(
        units::meter_t{globalY(GlobalOutput::kX)},
        units::meter_t{globalY(GlobalOutput::kY)}, globalTimestamp);

    xHat = drivetrain.GetStates();
    double xDiff2 = std::abs(xHat(State::kX) - x(State::kX));
    double yDiff2 = std::abs(xHat(State::kY) - x(State::kY));

    EXPECT_GT(xDiff1, xDiff2);
    EXPECT_GT(yDiff1, yDiff2);
}
