// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/Vision.hpp"

#include <chrono>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc2/Timer.h>
#include <units/angle.h>

#include "TargetModel.hpp"

using namespace frc3512;

const frc::Transform2d Vision::kCameraInGlobalToTurretInGlobal{
    frc::Pose2d{}, frc::Pose2d{0_in, -3_in, 0_rad}};

Vision::Vision() {
    m_listenerHandle =
        m_pose.AddListener([=](const auto&) { ProcessNewMeasurement(); },
                           NT_NOTIFY_NEW | NT_NOTIFY_UPDATE | NT_NOTIFY_LOCAL);
}

Vision::~Vision() { m_pose.RemoveListener(m_listenerHandle); }

std::optional<Vision::GlobalMeasurement> Vision::GetGlobalMeasurement() {
    return m_measurements.pop();
}

void Vision::ProcessNewMeasurement() {
    units::microsecond_t latency(
        static_cast<int64_t>(m_latency.GetDouble(-1) * 1000));

    // PnP data is transformation from camera's coordinate frame to target's
    // coordinate frame
    std::vector<double> pose = m_pose.GetDoubleArray({0.0, 0.0, 0.0});

    // If we don't see the target, don't push data into the queue
    if (pose == std::vector{0.0, 0.0, 0.0}) {
        return;
    }

    frc::Pose2d markerInGlobal = {0_m, 0_m, units::radian_t{0}};
    // The transformation from PnP data to the origin is from the camera's point
    // of view
    frc::Transform2d markerInGlobalToCameraInGlobal{
        frc::Pose2d{units::inch_t{pose[0]}, units::inch_t{pose[1]},
                    frc::Rotation2d{units::degree_t{pose[2]}}},
        frc::Pose2d{}};
    auto cameraInGlobal =
        markerInGlobal.TransformBy(markerInGlobalToCameraInGlobal);

    auto turretInGlobal =
        cameraInGlobal.TransformBy(kCameraInGlobalToTurretInGlobal);

    auto timestamp = frc2::Timer::GetFPGATimestamp();
    timestamp -= latency;

    m_measurements.push({timestamp, turretInGlobal});
}
