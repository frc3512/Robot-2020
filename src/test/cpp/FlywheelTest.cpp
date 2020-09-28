// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/Notifier.h>
#include <frc/simulation/SimHooks.h>
#include <frc2/Timer.h>
#include <gtest/gtest.h>
#include <units/math.h>

#include "RenameCSVs.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Flywheel.hpp"
#include "subsystems/Turret.hpp"
#include "subsystems/Vision.hpp"

TEST(FlywheelTest, ReachesGoal) {
    frc::sim::PauseTiming();

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
        // This noise simulates scheduling jitter. It's clamped to 0 so the next
        // Notifier still triggers.
        auto noise = units::math::min(
            0_s,
            units::second_t{frc::MakeWhiteNoiseVector(0.001)(0)} + 0.001_s);
        frc::sim::StepTiming(frc3512::Constants::kDt + noise);
    }

    frc3512::ControllerSubsystemBase::Disable();

    frc::sim::ResumeTiming();

    RenameCSVs("FlywheelTest", "./Flywheel ");

    EXPECT_TRUE(flywheel.AtGoal());
}
