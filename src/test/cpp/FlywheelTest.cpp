// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/Notifier.h>
#include <frc/simulation/SimHooks.h>
#include <frc2/Timer.h>
#include <gtest/gtest.h>

#include "CSVTestUtil.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Flywheel.hpp"
#include "subsystems/Turret.hpp"
#include "subsystems/Vision.hpp"

namespace {
class FlywheelTest : public testing::Test {
protected:
    void SetUp() override { frc::sim::PauseTiming(); }

    void TearDown() override { frc::sim::ResumeTiming(); }
};
}  // namespace

TEST_F(FlywheelTest, ReachesGoal) {
    frc3512::Vision vision;
    frc3512::Drivetrain drivetrain;
    frc3512::Turret turret{vision, drivetrain};
    frc3512::Flywheel flywheel{turret};

    frc3512::SubsystemBase::RunAllTeleopInit();
    frc3512::ControllerSubsystemBase::Enable();

    flywheel.SetGoal(500.0_rad_per_s);

    frc::Notifier teleopPeriodic{&frc3512::SubsystemBase::RunAllTeleopPeriodic};
    teleopPeriodic.StartPeriodic(20_ms);

    frc2::Timer timer;
    timer.Start();
    while (timer.Get() < 2_s) {
        frc::sim::StepTiming(frc3512::Constants::kDt);
    }

    frc3512::SubsystemBase::RunAllDisabledInit();
    frc3512::ControllerSubsystemBase::Disable();

    frc3512::AddPrefixToCSVs("FlywheelTest");

    EXPECT_TRUE(flywheel.AtGoal());
}
