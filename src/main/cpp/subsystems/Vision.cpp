// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/Vision.hpp"

#include <chrono>
#include <vector>

#include <frc/RobotBase.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc2/Timer.h>
#include <units/angle.h>

using namespace frc3512;

const frc::Transform2d Vision::kCameraInGlobalToTurretInGlobal{
    frc::Pose2d{}, frc::Pose2d{0_in, -3_in, 0_rad}};

void Vision::TurnLEDOn() { m_rpiCam.SetLEDMode(photonlib::LEDMode::kOn); }

void Vision::TurnLEDOff() { m_rpiCam.SetLEDMode(photonlib::LEDMode::kOff); }

bool Vision::IsLEDOn() const {
    return m_rpiCam.GetLEDMode() == photonlib::LEDMode::kOn;
}

void Vision::SubscribeToVisionData(
    frc3512::static_concurrent_queue<GlobalMeasurement, 8>& queue) {
    std::scoped_lock lock{m_mutex};
    m_subsystemQueues.push_back(&queue);
}

void Vision::ProcessNewMeasurement() {
    units::second_t latency = m_result.GetLatency();

    photonlib::PhotonTrackedTarget target = m_result.GetBestTarget();

    auto cameraToTargetInGlobal = target.GetCameraRelativePose();

    auto targetToCameraInGlobal =
        TargetModel::kTargetPoseInGlobal.TransformBy(cameraToTargetInGlobal);

    auto turretInGlobal =
        targetToCameraInGlobal.TransformBy(kCameraInGlobalToTurretInGlobal);

    std::array<double, 3> pose{
        turretInGlobal.X().to<double>(), turretInGlobal.Y().to<double>(),
        turretInGlobal.Rotation().Radians().to<double>()};
    m_hasPoseEntry.SetDoubleArray(pose);
    m_hasYawEntry.SetDouble(target.GetYaw());

    auto timestamp = frc2::Timer::GetFPGATimestamp();
    timestamp -= latency;

    std::scoped_lock lock{m_mutex};
    for (auto& queue : m_subsystemQueues) {
        queue->push({turretInGlobal, timestamp});
    }
}

void Vision::UpdateVisionMeasurementsSim(
    const frc::Pose2d& drivetrainPose,
    const frc::Rotation2d& cameraHeading) {
    m_simVision.ProcessFrame(drivetrainPose);

    using namespace Constants::Vision;
    m_simVision.MoveCamera(frc::Transform2d{frc::Translation2d{}, cameraHeading}, kCameraHeight,
                           kCameraPitch);
}

void Vision::SimulationInit() {
    frc::Pose2d kTargetPose{
        frc::Translation2d{TargetModel::kCenter.X(), TargetModel::kCenter.Y()} +
            TargetModel::kOffset,
        units::radian_t{wpi::math::pi}};
    photonlib::SimVisionTarget newTgt{
        kTargetPose, TargetModel::kC.Z(),
        TargetModel::kG.Y() - TargetModel::kA.Y(),
        TargetModel::kCenter.Z() - TargetModel::kC.Z()};
    m_simVision.AddSimVisionTarget(newTgt);
}

void Vision::RobotPeriodic() {
    m_result = m_rpiCam.GetLatestResult();

    if (m_result.HasTargets() &&
        !frc::DriverStation::GetInstance().IsDisabled()) {
        ProcessNewMeasurement();
        m_hasTargetEntry.SetBoolean(true);
    } else {
        m_hasTargetEntry.SetBoolean(false);
    }
}
