// Copyright (c) 2019-2020 FRC Team 3512. All Rights Reserved.

#include <frc/simulation/JoystickSim.h>
#include <frc/simulation/SimHooks.h>
#include <frc2/Timer.h>
#include <gtest/gtest.h>
#include <wpi/math>

#include "CSVTestUtil.hpp"
#include "Constants.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Turret.hpp"
#include "subsystems/Vision.hpp"

namespace {
class TurretTest : public testing::Test {
protected:
    void SetUp() override { frc::sim::PauseTiming(); }

    void TearDown() override { frc::sim::ResumeTiming(); }
};
}  // namespace

TEST_F(TurretTest, ConfigSpaceLimits) {
    using namespace frc3512::Constants::Robot;

    static constexpr int kPovCW = 90;
    static constexpr int kPovCCW = 270;

    frc::sim::JoystickSim appendageStick1{kAppendageStick1Port};

    frc3512::Vision vision;
    frc3512::Drivetrain drivetrain;
    frc3512::Turret turret{vision, drivetrain};
    turret.SetManualOverride(true);

    frc3512::SubsystemBase::RunAllTeleopInit();
    frc3512::ControllerSubsystemBase::Enable();

    frc::Notifier teleopPeriodic{&frc3512::SubsystemBase::RunAllTeleopPeriodic};
    teleopPeriodic.StartPeriodic(20_ms);

    // Verify turret can move CCW and CW when it isn't at the soft limits
    appendageStick1.SetPOV(kPovCW);
    frc::sim::StepTiming(5_ms);
    frc::sim::StepTiming(5_ms);
    frc::sim::StepTiming(5_ms);
    frc::sim::StepTiming(5_ms);
    EXPECT_LT(turret.GetMotorOutput(), 0_V);
    appendageStick1.SetPOV(kPovCCW);
    frc::sim::StepTiming(5_ms);
    frc::sim::StepTiming(5_ms);
    frc::sim::StepTiming(5_ms);
    frc::sim::StepTiming(5_ms);
    EXPECT_GT(turret.GetMotorOutput(), 0_V);

    // Move turret into CW limit
    while (!turret.HasPassedCWLimit()) {
        appendageStick1.SetPOV(kPovCW);
        frc::sim::StepTiming(5_ms);
        frc::sim::StepTiming(5_ms);
        frc::sim::StepTiming(5_ms);
        frc::sim::StepTiming(5_ms);
    }

    // Don't let turret move past CW limit
    appendageStick1.SetPOV(kPovCW);
    frc::sim::StepTiming(5_ms);
    frc::sim::StepTiming(5_ms);
    frc::sim::StepTiming(5_ms);
    frc::sim::StepTiming(5_ms);
    EXPECT_EQ(turret.GetMotorOutput(), 0_V);

    // Let turret move away from CW limit
    appendageStick1.SetPOV(kPovCCW);
    frc::sim::StepTiming(5_ms);
    frc::sim::StepTiming(5_ms);
    frc::sim::StepTiming(5_ms);
    frc::sim::StepTiming(5_ms);
    EXPECT_GT(turret.GetMotorOutput(), 0_V);

    // Move turret into CCW limit
    while (!turret.HasPassedCCWLimit()) {
        appendageStick1.SetPOV(kPovCCW);
        frc::sim::StepTiming(5_ms);
        frc::sim::StepTiming(5_ms);
        frc::sim::StepTiming(5_ms);
        frc::sim::StepTiming(5_ms);
    }

    // Don't let turret move past CCW limit
    appendageStick1.SetPOV(kPovCCW);
    frc::sim::StepTiming(5_ms);
    frc::sim::StepTiming(5_ms);
    frc::sim::StepTiming(5_ms);
    frc::sim::StepTiming(5_ms);
    EXPECT_EQ(turret.GetMotorOutput(), 0_V);

    // Let turret move away from CCW limit
    appendageStick1.SetPOV(kPovCW);
    frc::sim::StepTiming(5_ms);
    frc::sim::StepTiming(5_ms);
    frc::sim::StepTiming(5_ms);
    frc::sim::StepTiming(5_ms);
    EXPECT_LT(turret.GetMotorOutput(), 0_V);

    frc3512::SubsystemBase::RunAllDisabledInit();
    frc3512::ControllerSubsystemBase::Disable();
}

TEST(TurretTest, DISABLED_ReachesReferenceStaticDrivetrain) {
    frc3512::Vision vision;
    frc3512::Drivetrain drivetrain;
    frc3512::Turret turret{vision, drivetrain};
    turret.SetManualOverride(false);

    frc3512::SubsystemBase::RunAllAutonomousInit();
    frc3512::ControllerSubsystemBase::Enable();

    frc::Notifier autonomousPeriodic{
        &frc3512::SubsystemBase::RunAllAutonomousPeriodic};
    autonomousPeriodic.StartPeriodic(20_ms);

    frc2::Timer currentTime;
    currentTime.Start();
    while (currentTime.Get() < 10_s) {
        frc::sim::StepTiming(frc3512::Constants::kDt);
    }

    frc3512::SubsystemBase::RunAllDisabledInit();
    frc3512::ControllerSubsystemBase::Disable();

    frc3512::AddPrefixToCSVs("TurretTest Static");

    EXPECT_TRUE(turret.AtGoal());
}

TEST_F(TurretTest, DISABLED_ReachesReferenceRotateInPlaceDrivetrain) {
    // TODO: Make the drivetrain actually rotate in place instead of follow an
    // s-curve.

    frc3512::Vision vision;
    frc3512::Drivetrain drivetrain;
    frc3512::Turret turret{vision, drivetrain};
    turret.SetManualOverride(false);

    frc3512::SubsystemBase::RunAllAutonomousInit();
    frc3512::ControllerSubsystemBase::Enable();

    drivetrain.SetWaypoints(frc::Pose2d(0_m, 0_m, 0_rad), {},
                            frc::Pose2d(4.8768_m, 2.7432_m, 0_rad));

    frc::Notifier autonomousPeriodic{
        &frc3512::SubsystemBase::RunAllAutonomousPeriodic};
    autonomousPeriodic.StartPeriodic(20_ms);

    frc2::Timer currentTime;
    currentTime.Start();
    while (currentTime.Get() < 10_s) {
        frc::sim::StepTiming(frc3512::Constants::kDt);
    }

    frc3512::SubsystemBase::RunAllDisabledInit();
    frc3512::ControllerSubsystemBase::Disable();

    frc3512::AddPrefixToCSVs("TurretTest RotateInPlace");

    EXPECT_TRUE(turret.AtGoal());
}

TEST_F(TurretTest, DISABLED_ReachesReferenceSCurveDrivetrain) {
    frc3512::Vision vision;
    frc3512::Drivetrain drivetrain;
    frc3512::Turret turret{vision, drivetrain};
    turret.SetManualOverride(false);

    frc3512::SubsystemBase::RunAllAutonomousInit();
    frc3512::ControllerSubsystemBase::Enable();

    drivetrain.SetWaypoints(frc::Pose2d(0_m, 0_m, 0_rad), {},
                            frc::Pose2d(4.8768_m, 2.7432_m, 0_rad));

    frc::Notifier autonomousPeriodic{
        &frc3512::SubsystemBase::RunAllAutonomousPeriodic};
    autonomousPeriodic.StartPeriodic(20_ms);

    frc2::Timer currentTime;
    currentTime.Start();
    while (currentTime.Get() < 10_s) {
        frc::sim::StepTiming(frc3512::Constants::kDt);
    }

    frc3512::SubsystemBase::RunAllDisabledInit();
    frc3512::ControllerSubsystemBase::Disable();

    frc3512::AddPrefixToCSVs("TurretTest SCurve");

    EXPECT_TRUE(turret.AtGoal());
}

TEST_F(TurretTest, ReachesReferenceAutonDrivetrain) {
    frc3512::Vision vision;
    frc3512::Drivetrain drivetrain;
    frc3512::Turret turret{vision, drivetrain};
    turret.SetManualOverride(false);

    frc3512::SubsystemBase::RunAllAutonomousInit();
    frc3512::ControllerSubsystemBase::Enable();

    frc::Pose2d initialPose{12.65_m, 5.800_m, units::radian_t{wpi::math::pi}};
    drivetrain.SetWaypoints(
        initialPose, {},
        frc::Pose2d(12.65_m - frc3512::Drivetrain::kLength, 5.800_m,
                    units::radian_t{wpi::math::pi}));
    drivetrain.Reset(initialPose);

    frc::Notifier autonomousPeriodic{
        &frc3512::SubsystemBase::RunAllAutonomousPeriodic};
    autonomousPeriodic.StartPeriodic(20_ms);

    frc2::Timer currentTime;
    currentTime.Start();
    while (currentTime.Get() < 10_s) {
        frc::sim::StepTiming(frc3512::Constants::kDt);
    }

    frc3512::SubsystemBase::RunAllDisabledInit();
    frc3512::ControllerSubsystemBase::Disable();

    frc3512::AddPrefixToCSVs("TurretTest Auton");

    EXPECT_TRUE(turret.AtGoal());
}
