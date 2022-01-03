// Copyright (c) FRC Team 3512. All Rights Reserved.

#include <frc/Notifier.h>
#include <frc/simulation/SimHooks.h>
#include <gtest/gtest.h>

#include "Constants.hpp"
#include "SimulatorTest.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Flywheel.hpp"
#include "subsystems/Intake.hpp"

class IntakeTest : public frc3512::SimulatorTest {};

TEST_F(IntakeTest, Deploy) {
    frc3512::Drivetrain drivetrain;
    frc3512::Flywheel flywheel{drivetrain};
    frc3512::Intake intake{flywheel};

    frc3512::SubsystemBase::RunAllTeleopInit();
    frc::Notifier controllerPeriodic{[&] { intake.TeleopPeriodic(); }};
    controllerPeriodic.StartPeriodic(frc3512::Constants::kControllerPeriod);

    intake.Deploy();

    frc::sim::StepTiming(2_s);

    EXPECT_TRUE(intake.IsDeployed());

    frc::sim::StepTiming(2_s);

    intake.Stow();

    frc3512::SubsystemBase::RunAllDisabledInit();

    EXPECT_FALSE(intake.IsDeployed());
}
