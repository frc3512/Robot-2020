// Copyright (c) FRC Team 3512. All Rights Reserved.

#include <frc/Notifier.h>
#include <frc/simulation/SimHooks.h>
#include <gtest/gtest.h>

#include "Constants.hpp"
#include "SimulatorTest.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Flywheel.hpp"

class FlywheelTest : public frc3512::SimulatorTest {
public:
    frc3512::Drivetrain drivetrain;
    frc3512::Flywheel flywheel{drivetrain};
    frc::Notifier controllerPeriodic{[&] {
        flywheel.TeleopPeriodic();
        flywheel.ControllerPeriodic();
    }};

    FlywheelTest() {
        frc3512::SubsystemBase::RunAllTeleopInit();
        controllerPeriodic.StartPeriodic(frc3512::Constants::kControllerPeriod);
    }
};

TEST_F(FlywheelTest, ReachesGoal) {
    flywheel.SetGoal(500_rad_per_s);

    frc::sim::StepTiming(2_s);

    EXPECT_TRUE(flywheel.AtGoal());
}
