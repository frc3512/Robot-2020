// Copyright (c) 2019-2021 FRC Team 3512. All Rights Reserved.

#include <frc/simulation/DriverStationSim.h>
#include <gtest/gtest.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <units/math.h>

#include "Constants.hpp"
#include "NetworkTableUtil.hpp"
#include "SimulatorTest.hpp"
#include "TargetModel.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Flywheel.hpp"
#include "subsystems/Turret.hpp"
#include "subsystems/Vision.hpp"

class VisionTest : public frc3512::SimulatorTest {
public:
    frc3512::Drivetrain drivetrain;
    frc3512::Flywheel flywheel{drivetrain};
    frc3512::Turret turret{drivetrain, flywheel};
    frc3512::Vision vision{turret};
};

TEST_F(VisionTest, GetData) {
    photonlib::PhotonCamera rpiCam{frc3512::Vision::kCameraName};
    const frc::Pose2d drivetrainPose{12.89_m, 2.41_m,
                                     units::radian_t{wpi::numbers::pi}};

    // Simulation variables
    photonlib::SimVisionSystem simVision{frc3512::Vision::kCameraName,
                                         frc3512::Vision::kCameraDiagonalFOV,
                                         frc3512::Vision::kCameraPitch,
                                         frc::Transform2d{},
                                         frc3512::Vision::kCameraHeight,
                                         20_m,
                                         960,
                                         720,
                                         10};

    frc::Pose2d kTargetPose{
        frc::Translation2d{TargetModel::kCenter.X(), TargetModel::kCenter.Y()},
        0_rad};
    photonlib::SimVisionTarget newTgt{
        kTargetPose, TargetModel::kC.Z(),
        TargetModel::kB.Y() - TargetModel::kG.Y(),
        TargetModel::kCenter.Z() - TargetModel::kC.Z()};
    simVision.AddSimVisionTarget(newTgt);

    simVision.MoveCamera(frc::Transform2d{frc::Translation2d{},
                                          units::radian_t{wpi::numbers::pi}},
                         frc3512::Vision::kCameraHeight,
                         frc3512::Vision::kCameraPitch);
    simVision.ProcessFrame(drivetrainPose);

    // Flush and delay to ensure value propagates to tables
    nt::NetworkTableInstance::GetDefault().Flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    photonlib::PhotonPipelineResult result = rpiCam.GetLatestResult();

    ASSERT_TRUE(result.HasTargets());
}

TEST_F(VisionTest, QueueData) {
    // The vision subsystem only processes data when the robot is enabled
    frc::sim::DriverStationSim::SetEnabled(true);
    frc::sim::DriverStationSim::NotifyNewData();

    frc3512::SubsystemBase::RunAllSimulationInit();

    const frc::Pose2d drivetrainPose{12.89_m, 2.41_m,
                                     units::radian_t{wpi::numbers::pi}};
    frc3512::static_concurrent_queue<frc3512::Vision::GlobalMeasurement, 8>
        queue;

    vision.SubscribeToVisionData(queue);
    vision.UpdateVisionMeasurementsSim(
        drivetrainPose, frc::Transform2d{frc::Translation2d{},
                                         units::radian_t{wpi::numbers::pi}});

    frc3512::SubsystemBase::RunAllRobotPeriodic();

    // Flush and delay to ensure value propagates to tables
    nt::NetworkTableInstance::GetDefault().Flush();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    ASSERT_TRUE(queue.pop().has_value());

    vision.UnsubscribeFromVisionData(queue);
}
