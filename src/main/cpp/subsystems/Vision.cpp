// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/Vision.hpp"

#include <chrono>
#include <vector>

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc2/Timer.h>
#include <units/angle.h>

#include "subsystems/Drivetrain.hpp"
#include "subsystems/Turret.hpp"

using namespace frc3512;
using namespace Constants::Vision;

const frc::Transform2d Vision::kCameraInGlobalToTurretInGlobal{
    frc::Pose2d{}, frc::Pose2d{0_in, -3_in, 0_rad}};

frc::Transform2d operator+(const frc::Transform2d& first,
                           const frc::Transform2d& second) {
    return frc::Transform2d{
        frc::Pose2d{}, frc::Pose2d{}.TransformBy(first).TransformBy(second)};
}

Vision::Vision(Turret& turret) : m_turret(turret) {}

void Vision::TurnLEDOn() { m_rpiCam.SetLEDMode(photonlib::LEDMode::kOn); }

void Vision::TurnLEDOff() { m_rpiCam.SetLEDMode(photonlib::LEDMode::kOff); }

bool Vision::IsLEDOn() const {
    return m_rpiCam.GetLEDMode() == photonlib::LEDMode::kOn;
}

void Vision::SubscribeToVisionData(
    frc3512::static_concurrent_queue<GlobalMeasurement, 8>& queue) {
    std::scoped_lock lock{m_subsystemQueuesMutex};
    m_subsystemQueues.push_back(&queue);
}

void Vision::UnsubscribeFromVisionData(
    frc3512::static_concurrent_queue<GlobalMeasurement, 8>& queue) {
    std::scoped_lock lock{m_subsystemQueuesMutex};
    m_subsystemQueues.erase(
        std::remove(m_subsystemQueues.begin(), m_subsystemQueues.end(), &queue),
        m_subsystemQueues.end());
}

void Vision::UpdateVisionMeasurementsSim(
    const frc::Pose2d& drivetrainPose,
    const frc::Transform2d& turretInGlobalToDrivetrainInGlobal) {
    m_simVision.MoveCamera(turretInGlobalToDrivetrainInGlobal.Inverse() +
                               kCameraInGlobalToTurretInGlobal.Inverse(),
                           Constants::Vision::kCameraHeight,
                           Constants::Vision::kCameraPitch);
    m_simVision.ProcessFrame(drivetrainPose);
}

void Vision::SimulationInit() {
    m_simVision.MoveCamera(frc::Transform2d{}, kCameraHeight, kCameraPitch);

    frc::Pose2d kTargetPose{
        frc::Translation2d{TargetModel::kCenter.X(), TargetModel::kCenter.Y()},
        0_rad};
    photonlib::SimVisionTarget newTgt{
        kTargetPose, TargetModel::kC.Z(),
        TargetModel::kB.Y() - TargetModel::kG.Y(),
        TargetModel::kCenter.Z() - TargetModel::kC.Z()};
    m_simVision.AddSimVisionTarget(newTgt);
}

void Vision::RobotPeriodic() {
    const auto& result = m_rpiCam.GetLatestResult();

    if (result.GetTargets().size() == 0 ||
        frc::DriverStation::GetInstance().IsDisabled()) {
        return;
    }

    auto timestamp = frc2::Timer::GetFPGATimestamp() - result.GetLatency();

    photonlib::PhotonTrackedTarget target = result.GetBestTarget();

    // Converts solvePnP() data from the NetworkTables to a global drivetrain
    // pose measurement
    auto cameraInTarget = target.GetCameraRelativePose();
    auto cameraInGlobal =
        TargetModel::kTargetPoseInGlobal.TransformBy(cameraInTarget);
    auto drivetrainInGlobal =
        cameraInGlobal.TransformBy(kCameraInGlobalToTurretInGlobal)
            .TransformBy(m_turret.GetTurretInGlobalToDrivetrainInGlobal());

    std::array<double, 3> pose{
        drivetrainInGlobal.X().to<double>(),
        drivetrainInGlobal.Y().to<double>(),
        drivetrainInGlobal.Rotation().Radians().to<double>()};
    m_poseEntry.SetDoubleArray(pose);
    m_yawEntry.SetDouble(target.GetYaw());

    std::scoped_lock lock{m_subsystemQueuesMutex};
    for (auto& queue : m_subsystemQueues) {
        queue->push({drivetrainInGlobal, timestamp});
    }
}
