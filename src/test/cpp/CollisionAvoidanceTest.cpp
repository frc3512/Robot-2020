// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include <frc/Notifier.h>
#include <frc/simulation/JoystickSim.h>
#include <frc/simulation/SimHooks.h>
#include <frc2/Timer.h>
#include <gtest/gtest.h>

#include "Constants.hpp"
#include "HWConfig.hpp"
#include "SimulatorTest.hpp"
#include "subsystems/Climber.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Flywheel.hpp"
#include "subsystems/Turret.hpp"

class CollisionAvoidanceTest : public frc3512::SimulatorTest {
public:
    frc3512::Drivetrain drivetrain;
    frc3512::Flywheel flywheel{drivetrain};
    frc3512::Turret turret{drivetrain, flywheel};
    frc3512::Climber climber{turret};
};

TEST_F(CollisionAvoidanceTest, DISABLED_Avoidance) {
    frc2::Timer timer;
    timer.Start();

    frc::sim::JoystickSim appendageStick1{
        frc3512::HWConfig::kAppendageStick1Port};

    frc::Notifier controllerPeriodic{[&] {
        turret.TeleopPeriodic();
        turret.ControllerPeriodic();
        climber.RobotPeriodic();
        climber.TeleopPeriodic();
    }};
    frc3512::SubsystemBase::RunAllTeleopInit();
    controllerPeriodic.StartPeriodic(frc3512::Constants::kControllerPeriod);

    // Press the trigger button and ensure turret moves out of the way.
    appendageStick1.SetRawButton(1, true);
    appendageStick1.SetY(-1.0);
    frc::sim::StepTiming(20_ms);
    EXPECT_GT(turret.GetMotorOutput(), 0_V);

    frc::sim::StepTiming(2_s);

    // Move climber into top limit
    while (!climber.HasPassedTopLimit()) {
        appendageStick1.SetRawButton(1, true);
        appendageStick1.SetY(-1.0);
        frc::sim::StepTiming(20_ms);

        ASSERT_LT(timer.Get(), 30_s)
            << "Climber took too long to reach top limit";
    }

    // Ensure turret can't move into the climber's space
    turret.SetGoal(0_rad, 0_rad_per_s);
    frc::sim::StepTiming(500_ms);
    EXPECT_FALSE(turret.HasPassedCWLimit());

    // Move climber down
    while (!climber.HasPassedBottomLimit()) {
        appendageStick1.SetRawButton(1, true);
        appendageStick1.SetY(1.0);
        appendageStick1.NotifyNewData();
        frc::sim::StepTiming(20_ms);

        ASSERT_LT(timer.Get(), 30_s)
            << "Climber took too long to reach bottom limit";
    }

    // Ensure turret can move now
    appendageStick1.SetY(0.0);
    appendageStick1.SetRawButton(1, false);
    appendageStick1.NotifyNewData();
    frc::sim::StepTiming(20_ms);
    turret.SetGoal(0_rad, 0_rad_per_s);
    frc::sim::StepTiming(500_ms);
    EXPECT_LT(turret.GetMotorOutput(), 0_V);
}
