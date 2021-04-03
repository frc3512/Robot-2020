// Copyright (c) 2019-2021 FRC Team 3512. All Rights Reserved.

#include <frc2/Timer.h>
#include <gtest/gtest.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <units/math.h>

#include "SimulatorTest.hpp"
#include "TargetModel.hpp"
#include "subsystems/Vision.hpp"

#define EXPECT_NEAR_UNITS(val1, val2, eps) \
    EXPECT_LE(units::math::abs(val1 - val2), eps)

class VisionTest : public frc3512::SimulatorTest {
public:
    frc3512::Vision vision;
};

TEST_F(VisionTest, DISABLED_CalculateDrivetrainInGlobal) {
    /* frc3512::Vision vision;

    auto pose = frc3512::NetworkTableUtil::MakeDoubleArrayEntry(
        "photonvision/RPI-Cam/targetPose", {0, 0, 0});

    auto testMeasurement = [&](units::inch_t x, units::inch_t y,
                               units::degree_t theta, units::inch_t globalX,
                               units::inch_t globalY) {
        pose.SetDoubleArray({x.to<double>(), y.to<double>(),
                             theta.to<double>()});  // in, in, deg
        nt::NetworkTableInstance::GetDefault().Flush();

        // Delay to ensure value propagates to tables
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        auto globalMeasurement = vision.GetGlobalMeasurement();
        ASSERT_TRUE(globalMeasurement.has_value());
        EXPECT_NEAR_UNITS(globalMeasurement.value().pose.X(), globalX, 0.05_m);
        EXPECT_NEAR_UNITS(globalMeasurement.value().pose.Y(), globalY, 0.05_m);
    };

    frc::Translation2d kTargetCenter{TargetModel::kCenter.X(),
                                     TargetModel::kCenter.Y()};
    testMeasurement(5_m, 0_m, 0_deg,
                    kTargetCenter.X() - 5_m +
                        frc3512::Vision::kCameraInGlobalToTurretInGlobal.X(),
                    kTargetCenter.Y() - 0_m +
                        frc3512::Vision::kCameraInGlobalToTurretInGlobal.Y());
    testMeasurement(5_m, 0_m, 45_deg, 12.3935_m, 5.73816_m);
    testMeasurement(5_m, -2_m, -45_deg, 11.0871_m, 0.0813087_m); */
}

TEST_F(VisionTest, TestData) {
    photonlib::PhotonCamera rpiCam{"gloworm"};
    const frc::Pose2d drivetrainPose{620_in, 100_in, units::radian_t{0}};

    // Simulation variables
    using namespace frc3512::Constants::Vision;
    photonlib::SimVisionSystem simVision{
        "gloworm",
        kCameraDiagonalFOV,
        kCameraPitch,
        frc3512::Vision::kCameraInGlobalToTurretInGlobal,
        kCameraHeight,
        100_m,
        960,
        720,
        10};

    frc::Pose2d kTargetPose{
        frc::Translation2d{54_ft, (27.0_ft / 2) - 43.75_in - (48.0_in / 2.0)} +
            TargetModel::kOffset,
        frc::Rotation2d{0_rad}};
    photonlib::SimVisionTarget newTgt{kTargetPose, 81.91_in, 41.30_in - 6.70_in,
                                      98.19_in - 81.19_in};
    simVision.AddSimVisionTarget(newTgt);

    simVision.ProcessFrame(drivetrainPose);
    // Delay to ensure value propagates to tables
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    photonlib::PhotonPipelineResult result = rpiCam.GetLatestResult();

    ASSERT_TRUE(result.HasTargets());
    if (result.HasTargets()) {
        units::second_t latency = result.GetLatency();

        photonlib::PhotonTrackedTarget target = result.GetBestTarget();

        frc::Pose2d targetInGlobalToCameraInGlobal{
            target.GetCameraRelativePose().Translation(),
            target.GetCameraRelativePose().Rotation()};
        frc::Pose2d targetInGlobalToTurretInGlobal =
            targetInGlobalToCameraInGlobal.TransformBy(
                frc3512::Vision::kCameraInGlobalToTurretInGlobal);

        auto timestamp = frc2::Timer::GetFPGATimestamp();
        timestamp -= latency;
        EXPECT_NE(timestamp.to<double>(), 0);
        EXPECT_NE(targetInGlobalToTurretInGlobal.X().to<double>(), 0);
        EXPECT_NE(targetInGlobalToTurretInGlobal.Y().to<double>(), 0);
        EXPECT_NE(target.GetYaw(), 0);
    }
}

TEST_F(VisionTest, QueueData) {
    const frc::Pose2d drivetrainPose{620_in, 100_in,
                                     units::radian_t{wpi::math::pi}};
    frc3512::static_concurrent_queue<frc3512::Vision::GlobalMeasurement, 8>
        queue;

    vision.SubscribeToVisionData(queue);
    vision.UpdateVisionMeasurementsSim(drivetrainPose);

    frc3512::SubsystemBase::RunAllRobotPeriodic();

    // Delay to ensure value propagates to tables
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    ASSERT_TRUE(queue.pop().has_value());
}
