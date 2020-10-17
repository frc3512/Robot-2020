// Copyright (c) 2019-2021 FRC Team 3512. All Rights Reserved.

#include <fmt/format.h>
#include <frc/Notifier.h>
#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <gtest/gtest.h>
#include <units/acceleration.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/math>

#include "Constants.hpp"
#include "SimulatorTest.hpp"
#include "UnitsFormat.hpp"
#include "controllers/DrivetrainController.hpp"
#include "subsystems/Drivetrain.hpp"

class DrivetrainAutonomousTest : public frc3512::SimulatorTest {
public:
    frc3512::Drivetrain drivetrain;
    frc::Notifier controllerPeriodic{[&] {
        drivetrain.AutonomousPeriodic();
        drivetrain.ControllerPeriodic();
    }};

    DrivetrainAutonomousTest() {
        frc3512::SubsystemBase::RunAllAutonomousInit();
        controllerPeriodic.StartPeriodic(frc3512::Constants::kDt);
    }
};

class DrivetrainTeleopTest : public frc3512::SimulatorTest {
public:
    frc3512::Drivetrain drivetrain;
    frc::Notifier controllerPeriodic{[&] {
        drivetrain.TeleopPeriodic();
        drivetrain.ControllerPeriodic();
    }};

    DrivetrainTeleopTest() {
        frc3512::SubsystemBase::RunAllTeleopInit();
        controllerPeriodic.StartPeriodic(frc3512::Constants::kDt);
    }

    /**
     * Verifies Drivetrain::LimitAcceleration() outputs.
     *
     * @param leftVelocity       Left wheel velocity.
     * #param rightVelocity      Right wheel velocity.
     * @param leftInputVoltage   Unconstrained left motor voltage.
     * @param rightInputVoltage  Unconstrained right motor voltage.
     * @param maxAccel           Maximum acceleration.
     * @param leftOutputVoltage  Constrained left motor voltage.
     * @param rightOutputVoltage Constrained right motor voltage.
     */
    void VerifyLimitAcceleration(units::meters_per_second_t leftVelocity,
                                 units::meters_per_second_t rightVelocity,
                                 units::volt_t leftInput,
                                 units::volt_t rightInput,
                                 units::meters_per_second_squared_t maxAccel) {
        drivetrain.Reset({0_m, 0_m, 0_rad}, leftVelocity, rightVelocity);
        auto [leftOutput, rightOutput] =
            drivetrain.LimitAcceleration(leftInput, rightInput, maxAccel);

#if 0
        // Ensure left output is always smaller than left input with the same
        // sign
        if (leftInput > 0_V) {
            EXPECT_LE(leftOutput, leftInput)
                << leftVelocity << rightVelocity << leftInput << rightInput
                << maxAccel;
            EXPECT_GE(leftOutput, 0_V) << leftVelocity << rightVelocity
                                       << leftInput << rightInput << maxAccel;
        } else {
            EXPECT_GE(leftOutput, leftInput)
                << leftVelocity << rightVelocity << leftInput << rightInput
                << maxAccel;
            EXPECT_LE(leftOutput, 0_V) << leftVelocity << rightVelocity
                                       << leftInput << rightInput << maxAccel;
        }

        // Ensure right output is always smaller than right input with the same
        // sign
        if (rightInput > 0_V) {
            EXPECT_LE(rightOutput, rightInput);
            EXPECT_GE(rightOutput, 0_V);
        } else {
            EXPECT_GE(rightOutput, rightInput);
            EXPECT_LE(rightOutput, 0_V);
        }
#endif

        frc3512::DrivetrainController controller;
        auto plant = controller.GetPlant();

        Eigen::Matrix<double, 2, 1> x;
        x << leftVelocity.to<double>(), rightVelocity.to<double>();
        Eigen::Matrix<double, 2, 1> u;
        Eigen::Matrix<double, 2, 1> xdot;
        units::meters_per_second_squared_t a_l;
        units::meters_per_second_squared_t a_r;

        u << leftInput.to<double>(), rightInput.to<double>();
        xdot = plant.A() * x + plant.B() * u;
        a_l = units::meters_per_second_squared_t{xdot(0)};
        a_r = units::meters_per_second_squared_t{xdot(1)};
        auto oldAlpha = (a_r - a_l) / frc3512::DrivetrainController::kWidth;

        u << leftOutput.to<double>(), rightOutput.to<double>();
        xdot = plant.A() * x + plant.B() * u;
        a_l = units::meters_per_second_squared_t{xdot(0)};
        a_r = units::meters_per_second_squared_t{xdot(1)};
        auto newAlpha = (a_r - a_l) / frc3512::DrivetrainController::kWidth;

        EXPECT_NEAR(oldAlpha.to<double>(), newAlpha.to<double>(), 1e-6);

        // auto accel = (a_l + a_r) / 2.0;

        // EXPECT_GE(accel, -maxAccel);
        // EXPECT_LE(accel, maxAccel);

#if 0
        EXPECT_PRED_FORMAT2(testing::DoubleLE, -maxAccel.to<double>(),
                            accel.to<double>())
            << fmt::format("\tleft velocity = {}\n", leftVelocity)
            << fmt::format("\tright velocity = {}\n", rightVelocity)
            << fmt::format("\tleft input = {}\n", leftInput)
            << fmt::format("\tleft output = {}\n", leftOutput)
            << fmt::format("\tright input = {}\n", rightInput)
            << fmt::format("\tright output = {}\n", rightOutput)
            << fmt::format("\tmax acceleration = {}\n", maxAccel);
        EXPECT_PRED_FORMAT2(testing::DoubleLE, accel.to<double>(),
                            maxAccel.to<double>())
            << fmt::format("\tleft velocity = {}\n", leftVelocity)
            << fmt::format("\tright velocity = {}\n", rightVelocity)
            << fmt::format("\tleft input = {}\n", leftInput)
            << fmt::format("\tleft output = {}\n", leftOutput)
            << fmt::format("\tright input = {}\n", rightInput)
            << fmt::format("\tright output = {}\n", rightOutput)
            << fmt::format("\tmax acceleration = {}\n", maxAccel);
#endif
    }
};

TEST_F(DrivetrainAutonomousTest, ReachesReferenceStraight) {
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

TEST_F(DrivetrainAutonomousTest, ReachesReferenceCurve) {
    const frc::Pose2d kInitialPose{0_m, 0_m, 0_rad};

    drivetrain.Reset(kInitialPose);
    drivetrain.AddTrajectory(kInitialPose, {},
                             frc::Pose2d(4.8768_m, 2.7432_m, 0_rad));

    frc::sim::StepTiming(10_s);

    EXPECT_TRUE(drivetrain.AtGoal());
}

TEST_F(DrivetrainAutonomousTest, ReachesReferenceOffsetCurve) {
    const frc::Pose2d kInitialPose{5_m, 2_m, 0_rad};

    drivetrain.Reset(kInitialPose);
    drivetrain.AddTrajectory(kInitialPose, {},
                             frc::Pose2d(9.8768_m, 4.7432_m, 0_rad));

    frc::sim::StepTiming(10_s);

    EXPECT_TRUE(drivetrain.AtGoal());
}

TEST_F(DrivetrainAutonomousTest, TrajectoryQueue) {
    // Initial Pose - Right in line with the Target Zone
    const frc::Pose2d kInitialPose{12.89_m, 2.41_m,
                                   units::radian_t{wpi::math::pi}};
    // Mid Pose - Right before first/closest ball in the Trench Run
    const frc::Pose2d kMidPose{9.82_m + 0.5 * frc3512::Drivetrain::kLength,
                               0.705_m, units::radian_t{wpi::math::pi}};
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
    drivetrain.AddTrajectory({kInitialPose, kMidPose, kEndPose}, config);

    // Drive back
    auto config2 = frc3512::Drivetrain::MakeTrajectoryConfig();
    config2.SetReversed(true);
    drivetrain.AddTrajectory(kEndPose, {}, kMidPose, config2);

    frc::sim::StepTiming(10_s);

    EXPECT_TRUE(drivetrain.AtGoal());
}

TEST_F(DrivetrainAutonomousTest, CorrectsTowardGlobalY) {
    static constexpr bool kIdealModel = false;

    drivetrain.AddTrajectory(frc::Pose2d(0_m, 0_m, 0_rad), {},
                             frc::Pose2d(4.8768_m, 2.7432_m, 0_rad));

    // Ensure error covariance is nonzero
    frc::sim::StepTiming(10_s);

    Eigen::Matrix<double, 7, 1> x;
    x << 5.0, 5.0, 1.0, 1.0, 1.0, 1.0, 1.0;

    Eigen::Matrix<double, 4, 1> localY =
        frc3512::DrivetrainController::LocalMeasurementModel(
            x, Eigen::Matrix<double, 2, 1>::Zero());
    Eigen::Matrix<double, 2, 1> globalY =
        frc3512::DrivetrainController::GlobalMeasurementModel(
            x, Eigen::Matrix<double, 2, 1>::Zero());
    auto globalTimestamp = frc2::Timer::GetFPGATimestamp();

    // Add measurement noise
    if constexpr (!kIdealModel) {
        localY += frc::MakeWhiteNoiseVector(0.0001, 0.005, 0.005, 7.0);
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

template <typename T>
std::vector<T> Range(T begin, T end, T step = T{1.0}) {
    std::vector<T> range;
    for (T i = begin; i < end; i += step) {
        range.emplace_back(i);
    }
    return range;
}

template <typename T>
std::vector<T> Range(T end) {
    return Range(T{0.0}, end);
}

TEST_F(DrivetrainTeleopTest, LimitAcceleration) {
    for (auto leftVelocity : Range(0_mps, 3.5_mps, 0.5_mps)) {
        for (auto rightVelocity : Range(0_mps, 3.5_mps, 0.5_mps)) {
            for (auto leftVoltage : Range(-12_V, 12_V, 0.5_V)) {
                for (auto rightVoltage : Range(-12_V, 12_V, 0.5_V)) {
                    VerifyLimitAcceleration(leftVelocity, rightVelocity,
                                            leftVoltage, rightVoltage,
                                            4.5_mps_sq);
                }
            }
        }
    }

    // Not moving; accelerate in straight line
    VerifyLimitAcceleration(0_mps, 0_mps, 0_V, 0_V, 4.5_mps_sq);
    VerifyLimitAcceleration(0_mps, 0_mps, 2_V, 2_V, 4.5_mps_sq);
    VerifyLimitAcceleration(0_mps, 0_mps, 4_V, 4_V, 4.5_mps_sq);
    VerifyLimitAcceleration(0_mps, 0_mps, 6_V, 6_V, 4.5_mps_sq);
    VerifyLimitAcceleration(0_mps, 0_mps, 8_V, 8_V, 4.5_mps_sq);
    VerifyLimitAcceleration(0_mps, 0_mps, 12_V, 12_V, 4.5_mps_sq);

    // Not moving; accelerate CCW
    VerifyLimitAcceleration(0_mps, 0_mps, 0_V, 0_V, 4.5_mps_sq);
    VerifyLimitAcceleration(0_mps, 0_mps, 2_V, 2_V, 4.5_mps_sq);
    VerifyLimitAcceleration(0_mps, 0_mps, 4_V, 4_V, 4.5_mps_sq);
    VerifyLimitAcceleration(0_mps, 0_mps, 6_V, 6_V, 4.5_mps_sq);
    VerifyLimitAcceleration(0_mps, 0_mps, 8_V, 8_V, 4.5_mps_sq);
    VerifyLimitAcceleration(0_mps, 0_mps, 12_V, 12_V, 4.5_mps_sq);
}
