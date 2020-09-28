// Copyright (c) 2019-2020 FRC Team 3512. All Rights Reserved.

#include <frc/simulation/RoboRioSim.h>
#include <frc/simulation/SimHooks.h>
#include <frc/system/plant/DCMotor.h>
#include <frc2/Timer.h>
#include <gtest/gtest.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>
#include <wpi/MathExtras.h>
#include <wpi/math>

#include "Constants.hpp"
#include "RenameCSVs.hpp"
#include "controllers/DrivetrainController.hpp"
#include "controllers/TurretController.hpp"
#include "subsystems/Drivetrain.hpp"

#define EXPECT_NEAR_UNITS(val1, val2, eps) \
    EXPECT_LE(units::math::abs((val1) - (val2)), eps)

static constexpr bool kIdealModel = true;

void RunSimulation(
    frc3512::DrivetrainController& drivetrainController,
    frc3512::TurretController& turretController,
    Eigen::Matrix<double, 10, 1> drivetrainX =
        Eigen::Matrix<double, 10, 1>::Zero(),
    Eigen::Matrix<double, 2, 1> turretX = Eigen::Matrix<double, 2, 1>::Zero()) {
    using frc3512::Constants::kDt;

    Eigen::Matrix<double, 2, 1> drivetrainU =
        Eigen::Matrix<double, 2, 1>::Zero();
    Eigen::Matrix<double, 1, 1> turretU = Eigen::Matrix<double, 1, 1>::Zero();

    frc::sim::PauseTiming();

    frc2::Timer currentTime;
    currentTime.Start();
    while (currentTime.Get() < 10_s) {
        auto dt = kDt;
        if constexpr (!kIdealModel) {
            dt += units::second_t{frc::MakeWhiteNoiseVector(0.001)(0)};
        }

        // Update drivetrain controller
        Eigen::Matrix<double, 3, 1> drivetrainY =
            frc3512::DrivetrainController::LocalMeasurementModel(
                drivetrainX, Eigen::Matrix<double, 2, 1>::Zero());
        // Add measurement noise to drivetrain controller
        if constexpr (!kIdealModel) {
            drivetrainY += frc::MakeWhiteNoiseVector(0.0001, 0.005, 0.005);
        }
        drivetrainController.SetMeasuredLocalOutputs(
            units::radian_t{drivetrainY(0)}, units::meter_t{drivetrainY(1)},
            units::meter_t{drivetrainY(2)});
        drivetrainController.Update(kDt, frc2::Timer::GetFPGATimestamp());

        // Update turret controller
        turretController.SetDrivetrainStatus(drivetrainX);
        turretController.SetMeasuredOutputs(units::radian_t{turretX(0)});
        turretController.Update(kDt, frc2::Timer::GetFPGATimestamp());

        drivetrainU = drivetrainController.GetInputs();
        turretU = turretController.GetInputs();

        // Account for battery voltage drop due to current draw for the turret
        if constexpr (!kIdealModel) {
            constexpr auto Vbat = 12_V;
            constexpr auto Rbat = 0.03_Ohm;
            constexpr auto r = 0.0746125_m;  // Drivetrain wheel radius

            using Input = frc3512::DrivetrainController::Input;
            using State = frc3512::DrivetrainController::State;
            constexpr auto turretMotor = frc::DCMotor::NEO(1);
            units::ampere_t turretLoad =
                turretMotor.Current(units::radians_per_second_t{turretX(1)},
                                    units::volt_t{turretU(0)}) *
                wpi::sgn(turretU(0));
            constexpr auto drivetrainMotors = frc::DCMotor::NEO(2);
            units::ampere_t loadIleft =
                drivetrainMotors.Current(
                    units::meters_per_second_t{
                        drivetrainX(State::kLeftVelocity)} /
                        r * 1_rad,
                    units::volt_t{drivetrainU(Input::kLeftVoltage)}) *
                wpi::sgn(drivetrainU(Input::kLeftVoltage));
            units::ampere_t loadIright =
                drivetrainMotors.Current(
                    units::meters_per_second_t{
                        drivetrainX(State::kRightVelocity)} /
                        r * 1_rad,
                    units::volt_t{drivetrainU(Input::kRightVoltage)}) *
                wpi::sgn(drivetrainU(Input::kRightVoltage));
            units::volt_t vLoaded =
                Vbat - turretLoad * Rbat - loadIleft * Rbat - loadIright * Rbat;
            double dsVoltage =
                vLoaded.to<double>() + frc::MakeWhiteNoiseVector(0.1)(0);
            frc::sim::RoboRioSim::SetVInVoltage(dsVoltage);

            drivetrainU *= dsVoltage / 12.0;
            turretU *= dsVoltage / 12.0;
        }

        drivetrainX = frc::RungeKutta(frc3512::DrivetrainController::Dynamics,
                                      drivetrainX, drivetrainU, dt);
        turretX = turretController.GetPlant().CalculateX(turretX, turretU, dt);
        frc::sim::StepTiming(dt);
    }

    frc::sim::ResumeTiming();
}

TEST(TurretTest, ReachesReferenceStaticDrivetrain) {
    // Initialize drivetrain controller
    frc3512::DrivetrainController drivetrainController;
    drivetrainController.Enable();

    // Initialize turret controller
    frc3512::TurretController turretController;
    turretController.Enable();

    RunSimulation(drivetrainController, turretController);

    RenameCSVs("TurretTest Static", "./Turret ");

    EXPECT_TRUE(turretController.AtGoal());
}

TEST(TurretTest, DISABLED_ReachesReferenceRotateInPlaceDrivetrain) {
    // TODO: Make the drivetrain actually rotate in place instead of follow an
    // s-curve.

    // Initialize drivetrain controller
    frc3512::DrivetrainController drivetrainController;
    drivetrainController.Enable();
    drivetrainController.SetWaypoints(frc::Pose2d(0_m, 0_m, 0_rad), {},
                                      frc::Pose2d(4.8768_m, 2.7432_m, 0_rad));

    // Initialize turret controller
    frc3512::TurretController turretController;
    turretController.Enable();

    RunSimulation(drivetrainController, turretController);

    RenameCSVs("TurretTest RotateInPlace", "./Drivetrain ");
    RenameCSVs("TurretTest RotateInPlace", "./Turret ");

    EXPECT_TRUE(turretController.AtGoal());
}

TEST(TurretTest, ReachesReferenceSCurveDrivetrain) {
    // Initialize drivetrain controller
    frc3512::DrivetrainController drivetrainController;
    drivetrainController.Enable();
    drivetrainController.SetWaypoints(frc::Pose2d(0_m, 0_m, 0_rad), {},
                                      frc::Pose2d(4.8768_m, 2.7432_m, 0_rad));

    // Initialize turret controller
    frc3512::TurretController turretController;
    turretController.Enable();

    RunSimulation(drivetrainController, turretController);

    RenameCSVs("TurretTest SCurve", "./Drivetrain ");
    RenameCSVs("TurretTest SCurve", "./Turret ");

    EXPECT_TRUE(turretController.AtGoal());
}

TEST(TurretTest, ReachesReferenceAutonDrivetrain) {
    frc::Pose2d initialPose{12.65_m, 5.800_m, units::radian_t{wpi::math::pi}};

    // Initialize drivetrain controller
    frc3512::DrivetrainController drivetrainController;
    drivetrainController.Reset(initialPose, initialPose);
    drivetrainController.Enable();
    drivetrainController.SetWaypoints(
        initialPose, {},
        frc::Pose2d(12.65_m - frc3512::Drivetrain::kLength, 5.800_m,
                    units::radian_t{wpi::math::pi}));

    // Initialize turret controller
    frc3512::TurretController turretController;
    turretController.Enable();

    Eigen::Matrix<double, 10, 1> drivetrainX =
        Eigen::Matrix<double, 10, 1>::Zero();
    drivetrainX(2) = wpi::math::pi;

    RunSimulation(drivetrainController, turretController, drivetrainX);

    RenameCSVs("TurretTest Auton", "./Drivetrain ");
    RenameCSVs("TurretTest Auton", "./Turret ");

    EXPECT_TRUE(turretController.AtGoal());
}
