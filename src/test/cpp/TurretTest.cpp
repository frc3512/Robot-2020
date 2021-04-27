// Copyright (c) 2019-2021 FRC Team 3512. All Rights Reserved.

#include <frc/Notifier.h>
#include <frc/simulation/JoystickSim.h>
#include <frc/simulation/SimHooks.h>
#include <frc2/Timer.h>
#include <gtest/gtest.h>
#include <wpi/math>

#include "Constants.hpp"
#include "RealTimeRobot.hpp"
#include "SimulatorTest.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Flywheel.hpp"
#include "subsystems/Turret.hpp"
#include "subsystems/Vision.hpp"

class TurretTest : public frc3512::SimulatorTest {
public:
    frc3512::Vision vision;
    frc3512::Drivetrain drivetrain;
    frc3512::Flywheel flywheel{drivetrain};
    frc3512::Turret turret{vision, drivetrain, flywheel};
};

TEST_F(TurretTest, ConfigSpaceLimits) {
    using namespace frc3512::Constants::Robot;

    constexpr int kPovCW = 90;
    constexpr int kPovCCW = 270;

    frc2::Timer timer;
    timer.Start();

    frc::sim::JoystickSim appendageStick1{kAppendageStick1Port};

    frc3512::SubsystemBase::RunAllTeleopInit();
    turret.SetControlMode(frc3512::TurretController::ControlMode::kManual);
    frc::Notifier controllerPeriodic{[&] {
        turret.TeleopPeriodic();
        turret.ControllerPeriodic();
    }};
    controllerPeriodic.StartPeriodic(
        frc3512::RealTimeRobot::kDefaultControllerPeriod);

    // Verify turret can move CW when it isn't at the soft limits
    appendageStick1.SetPOV(kPovCW);
    frc::sim::StepTiming(20_ms);
    EXPECT_LT(turret.GetMotorOutput(), 0_V);

    // Verify turret can move CCW when it isn't at the soft limits
    appendageStick1.SetPOV(kPovCCW);
    frc::sim::StepTiming(20_ms);
    EXPECT_GT(turret.GetMotorOutput(), 0_V);

    // Move turret into CW limit
    while (!turret.HasPassedCWLimit()) {
        appendageStick1.SetPOV(kPovCW);
        frc::sim::StepTiming(20_ms);

        ASSERT_LT(timer.Get(), 30_s)
            << "Turret took too long to reach CW limit";
    }

    // Don't let turret move past CW limit
    appendageStick1.SetPOV(kPovCW);
    frc::sim::StepTiming(20_ms);
    EXPECT_EQ(turret.GetMotorOutput(), 0_V);

    // Let turret move away from CW limit
    appendageStick1.SetPOV(kPovCCW);
    frc::sim::StepTiming(20_ms);
    EXPECT_GT(turret.GetMotorOutput(), 0_V);

    // Move turret into CCW limit
    while (!turret.HasPassedCCWLimit()) {
        appendageStick1.SetPOV(kPovCCW);
        frc::sim::StepTiming(20_ms);

        ASSERT_LT(timer.Get(), 30_s)
            << "Turret took too long to reach CCW limit";
    }

    // Don't let turret move past CCW limit
    appendageStick1.SetPOV(kPovCCW);
    frc::sim::StepTiming(20_ms);
    EXPECT_EQ(turret.GetMotorOutput(), 0_V);

    // Let turret move away from CCW limit
    appendageStick1.SetPOV(kPovCW);
    frc::sim::StepTiming(20_ms);
    EXPECT_LT(turret.GetMotorOutput(), 0_V);
}

TEST_F(TurretTest, ReachesReferenceStaticDrivetrain) {
    turret.SetControlMode(frc3512::TurretController::ControlMode::kAutoAim);

    frc3512::SubsystemBase::RunAllAutonomousInit();
    frc::Notifier controllerPeriodic{[&] {
        drivetrain.AutonomousPeriodic();
        turret.AutonomousPeriodic();
        drivetrain.ControllerPeriodic();
        turret.ControllerPeriodic();
    }};
    controllerPeriodic.StartPeriodic(
        frc3512::RealTimeRobot::kDefaultControllerPeriod);

    frc::Pose2d initialPose{12.65_m, 5.800_m - 0.343_m,
                            units::radian_t{wpi::math::pi}};
    drivetrain.Reset(initialPose);

    frc::sim::StepTiming(10_s);

    EXPECT_TRUE(turret.AtGoal());
}

TEST_F(TurretTest, ReachesReferenceSCurveDrivetrain) {
    turret.SetControlMode(frc3512::TurretController::ControlMode::kAutoAim);

    frc3512::SubsystemBase::RunAllAutonomousInit();
    frc::Notifier controllerPeriodic{[&] {
        drivetrain.AutonomousPeriodic();
        turret.AutonomousPeriodic();
        drivetrain.ControllerPeriodic();
        turret.ControllerPeriodic();
    }};
    controllerPeriodic.StartPeriodic(
        frc3512::RealTimeRobot::kDefaultControllerPeriod);

    frc::Pose2d initialPose{4.8768_m, 2.7432_m, units::radian_t{wpi::math::pi}};

    drivetrain.Reset(initialPose);
    drivetrain.AddTrajectory(
        initialPose, {}, frc::Pose2d(0_m, 0_m, units::radian_t{wpi::math::pi}));

    frc::sim::StepTiming(10_s);

    EXPECT_TRUE(turret.AtGoal());
}

TEST_F(TurretTest, ReachesReferenceAutonDrivetrain) {
    turret.SetControlMode(frc3512::TurretController::ControlMode::kAutoAim);

    frc3512::SubsystemBase::RunAllAutonomousInit();
    frc::Notifier controllerPeriodic{[&] {
        drivetrain.AutonomousPeriodic();
        turret.AutonomousPeriodic();
        drivetrain.ControllerPeriodic();
        turret.ControllerPeriodic();
    }};
    controllerPeriodic.StartPeriodic(
        frc3512::RealTimeRobot::kDefaultControllerPeriod);

    frc::Pose2d initialPose{12.65_m, 5.800_m, units::radian_t{wpi::math::pi}};
    drivetrain.Reset(initialPose);
    drivetrain.AddTrajectory(
        initialPose, {},
        frc::Pose2d(12.65_m - frc3512::Drivetrain::kLength, 5.800_m,
                    units::radian_t{wpi::math::pi}));

    frc::sim::StepTiming(10_s);

    EXPECT_TRUE(turret.AtGoal());
}
