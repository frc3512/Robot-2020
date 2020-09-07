// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Vision.hpp"

#include <chrono>
#include <vector>

#include <frc/RobotBase.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>

#include "TargetModel.hpp"
#include "subsystems/Turret.hpp"

using namespace frc3512;

const frc::Transform2d Vision::kCameraInGlobalToTurretInGlobal{
    frc::Pose2d{}, frc::Pose2d{0_in, -1.0 * 3_in, 0_rad}};

Vision::Vision() {
    auto inst = nt::NetworkTableInstance::GetDefault();
#ifdef RUNNING_FRC_TESTS
    inst.StartLocal();
#else
    inst.StartClient();
#endif

    auto ledTable = inst.GetTable("LED Ring Light");
    m_ledIsOn = ledTable->GetEntry("LED-State");

    auto poseTable = inst.GetTable("chameleon-vision");
    auto rpiTable = poseTable->GetSubTable("RPI-Cam");
    m_pose = rpiTable->GetEntry("target-Pose");
    m_pose.AddListener(std::bind(&Vision::ProcessNewMeasurement, this),
                       NT_NOTIFY_NEW | NT_NOTIFY_UPDATE | NT_NOTIFY_LOCAL);
    m_latency = rpiTable->GetEntry("latency");
}

void Vision::TurnLEDOn() { m_ledIsOn.SetBoolean(true); }

void Vision::TurnLEDOff() { m_ledIsOn.SetBoolean(false); }

bool Vision::IsLEDOn() const { return !m_ledIsOn.GetBoolean(false); }

std::optional<Vision::GlobalMeasurement> Vision::GetGlobalMeasurement() {
    return m_measurements.pop();
}

void Vision::ProcessNewMeasurement() {
    namespace chrono = std::chrono;

    auto latency = static_cast<int64_t>(m_latency.GetDouble(-1) * 1000);

    // PnP data is transformation from camera's coordinate frame to target's
    // coordinate frame
    std::vector<double> pose = m_pose.GetDoubleArray({0.0, 0.0, 0.0});

    // If we don't see the target, don't push data into the queue
    if (pose == std::vector{0.0, 0.0, 0.0}) {
        return;
    }

    frc::Pose2d targetInGlobal{TargetModel::kCenter.X(),
                               TargetModel::kCenter.Y(), 0_rad};

    // The transformation from PnP data to the origin is from the camera's point
    // of view
    frc::Transform2d targetInGlobalToCameraInGlobal{
        frc::Pose2d{units::inch_t{pose[0]}, units::inch_t{pose[1]},
                    frc::Rotation2d{units::degree_t{pose[2]}}},
        frc::Pose2d{}};
    auto cameraInGlobal =
        targetInGlobal.TransformBy(targetInGlobalToCameraInGlobal);

    auto turretInGlobal =
        cameraInGlobal.TransformBy(kCameraInGlobalToTurretInGlobal);

    auto timestamp = chrono::steady_clock::now().time_since_epoch();
    timestamp -= chrono::microseconds{latency};
    auto timestampInt =
        chrono::duration_cast<chrono::microseconds>(timestamp).count();

    m_measurements.push({timestampInt, turretInGlobal});
}
