// Copyright (c) FRC Team 3512. All Rights Reserved.

#include <frc/Notifier.h>
#include <frc/Timer.h>
#include <frc/simulation/JoystickSim.h>
#include <frc/simulation/SimHooks.h>
#include <gtest/gtest.h>

#include "Constants.hpp"
#include "HWConfig.hpp"
#include "SimulatorTest.hpp"
#include "subsystems/Climber.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Flywheel.hpp"
#include "subsystems/Turret.hpp"

#define EXPECT_NEAR_UNITS(val1, val2, eps) \
    EXPECT_LE(units::math::abs(val1 - val2), eps)

class ClimberTest : public frc3512::SimulatorTest {
public:
    frc3512::Drivetrain drivetrain;
    frc3512::Flywheel flywheel{drivetrain};
    frc3512::Turret turret{drivetrain, flywheel};
    frc3512::Climber climber{turret};
};

TEST_F(ClimberTest, ConfigSpaceLimits) {
    frc::Timer timer;
    timer.Start();

    frc::sim::JoystickSim appendageStick1{
        frc3512::HWConfig::kAppendageStick1Port};

    // Make sure turret doesn't interfere with climber movement
    turret.Reset(units::radian_t{wpi::numbers::pi / 2.0});

    frc3512::SubsystemBase::RunAllTeleopInit();
    frc::Notifier controllerPeriodic{[&] {
        climber.RobotPeriodic();
        climber.TeleopPeriodic();
    }};
    controllerPeriodic.StartPeriodic(frc3512::Constants::kControllerPeriod);

    // Verify climber can't move below bottom soft limit
    appendageStick1.SetRawButton(1, true);
    appendageStick1.SetY(1.0);
    appendageStick1.NotifyNewData();
    frc::sim::StepTiming(20_ms);
    EXPECT_EQ(climber.GetElevatorMotorOutput(), 0_V);

    frc::sim::StepTiming(2_s);

    // Move climber into top limit
    while (!climber.HasPassedTopLimit()) {
        appendageStick1.SetRawButton(1, true);
        appendageStick1.SetY(-1.0);
        appendageStick1.NotifyNewData();
        frc::sim::StepTiming(20_ms);

        ASSERT_LT(timer.Get(), 30_s)
            << "Climber took too long to reach top limit";
    }

    // Don't let climber move past top limit
    appendageStick1.SetRawButton(1, true);
    appendageStick1.SetY(-1.0);
    appendageStick1.NotifyNewData();
    frc::sim::StepTiming(20_ms);
    EXPECT_EQ(climber.GetElevatorMotorOutput(), 0_V);
}
