// Copyright (c) FRC Team 3512. All Rights Reserved.

#include <frc/Notifier.h>
#include <frc/simulation/SimHooks.h>
#include <gtest/gtest.h>

#include "Constants.hpp"
#include "SimulatorTest.hpp"
#include "subsystems/Drivetrain.hpp"

class ModeTest : public frc3512::SimulatorTest {};

TEST_F(ModeTest, AutonToTeleop) {
    frc3512::Drivetrain drivetrain;

    frc3512::SubsystemBase::RunAllAutonomousInit();
    frc::Notifier autonomousPeriodic{[&] { drivetrain.ControllerPeriodic(); }};
    autonomousPeriodic.StartPeriodic(frc3512::Constants::kControllerPeriod);

    const frc::Pose2d kInitialPose{5_m, 0_m, 0_rad};
    drivetrain.Reset(kInitialPose);
    drivetrain.AddTrajectory(kInitialPose, {}, frc::Pose2d{15_m, 0_m, 0_rad});

    frc::sim::StepTiming(3_s);

    EXPECT_GT(drivetrain.GetInputs()(0), 0.0);
    EXPECT_GT(drivetrain.GetInputs()(1), 0.0);

    autonomousPeriodic.Stop();

    frc3512::SubsystemBase::RunAllTeleopInit();
    frc::Notifier teleopPeriodic{[&] {
        drivetrain.TeleopPeriodic();
        drivetrain.ControllerPeriodic();
    }};
    teleopPeriodic.StartPeriodic(frc3512::Constants::kControllerPeriod);

    frc::sim::StepTiming(20_ms);

    EXPECT_EQ(drivetrain.GetInputs()(0), 0.0);
    EXPECT_EQ(drivetrain.GetInputs()(1), 0.0);

    frc3512::SubsystemBase::RunAllDisabledInit();
}
