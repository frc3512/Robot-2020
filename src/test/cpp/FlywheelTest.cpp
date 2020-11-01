// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/Notifier.h>
#include <frc/simulation/SimHooks.h>
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
    {
        frc3512::Vision vision;
        frc3512::Drivetrain drivetrain;
        frc3512::Turret turret{vision, drivetrain};
        frc3512::Flywheel flywheel{turret};

        frc3512::SubsystemBase::RunAllTeleopInit();
        frc::Notifier controllerPeriodic{[&] {
            flywheel.TeleopPeriodic();
            flywheel.ControllerPeriodic();
        }};
        controllerPeriodic.StartPeriodic(frc3512::Constants::kDt);

        flywheel.SetGoal(500_rad_per_s);

        frc::sim::StepTiming(2_s);

        frc3512::SubsystemBase::RunAllDisabledInit();

        EXPECT_TRUE(flywheel.AtGoal());
    }

    frc3512::AddPrefixToCSVs("FlywheelTest");
}
