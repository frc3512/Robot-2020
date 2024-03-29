// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "subsystems/Vision.hpp"

#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/Timer.h>
#include <photonlib/PhotonUtils.h>

#include "TargetModel.hpp"
#include "subsystems/Turret.hpp"

using namespace frc3512;

const frc::Transform2d Vision::kCameraInGlobalToTurretInGlobal{
    frc::Pose2d{}, frc::Pose2d{0_in, -3_in, 0_rad}};

Vision::Vision(Turret& turret) : m_turret(turret) {}

void Vision::TurnLEDOn() { m_rpiCam.SetLEDMode(photonlib::LEDMode::kOn); }

void Vision::TurnLEDOff() { m_rpiCam.SetLEDMode(photonlib::LEDMode::kOff); }

bool Vision::IsLEDOn() const {
    return m_rpiCam.GetLEDMode() == photonlib::LEDMode::kOn;
}

void Vision::SubscribeToVisionData(
    wpi::static_circular_buffer<GlobalMeasurement, 8>& queue) {
    m_subsystemQueues.push_back(&queue);
}

void Vision::UnsubscribeFromVisionData(
    wpi::static_circular_buffer<GlobalMeasurement, 8>& queue) {
    m_subsystemQueues.erase(
        std::remove(m_subsystemQueues.begin(), m_subsystemQueues.end(), &queue),
        m_subsystemQueues.end());
}

void Vision::UpdateVisionMeasurementsSim(
    const frc::Pose2d& drivetrainPose,
    const frc::Transform2d& turretInGlobalToDrivetrainInGlobal) {
    m_simVision.MoveCamera(turretInGlobalToDrivetrainInGlobal.Inverse() +
                               kCameraInGlobalToTurretInGlobal.Inverse(),
                           kCameraHeight, kCameraPitch);
    m_simVision.ProcessFrame(drivetrainPose);
}

bool Vision::IsTargetDetected() const { return m_isTargetDetected; }

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
    static frc::Joystick appendageStick2{HWConfig::kAppendageStick2Port};

    if (appendageStick2.GetRawButtonPressed(5)) {
        TurnLEDOn();
    }

    const auto& result = m_rpiCam.GetLatestResult();

    if (result.GetTargets().size() == 0 || frc::DriverStation::IsDisabled()) {
        m_isTargetDetected = false;
        return;
    }

    m_isTargetDetected = true;
    auto timestamp = frc::Timer::GetFPGATimestamp() - result.GetLatency();

    photonlib::PhotonTrackedTarget target = result.GetBestTarget();

    // Converts solvePnP() data from the NetworkTables to a global drivetrain
    // pose measurement
    auto cameraInTarget = target.GetCameraRelativePose();
    auto cameraInGlobal =
        TargetModel::kTargetPoseInGlobal.TransformBy(cameraInTarget);
    auto drivetrainInGlobal =
        cameraInGlobal.TransformBy(kCameraInGlobalToTurretInGlobal)
            .TransformBy(m_turret.GetTurretInGlobalToDrivetrainInGlobal());

    std::array<double, 3> pose{drivetrainInGlobal.X().value(),
                               drivetrainInGlobal.Y().value(),
                               drivetrainInGlobal.Rotation().Radians().value()};
    m_poseEntry.SetDoubleArray(pose);

    m_pitch = units::degree_t{target.GetPitch()};
    m_yaw = units::degree_t{target.GetYaw()};

    m_yawEntry.SetDouble(units::radian_t{m_yaw}.value());

    units::meter_t range = photonlib::PhotonUtils::CalculateDistanceToTarget(
        kCameraHeight, TargetModel::kCenter.Z(), kCameraPitch,
        units::degree_t{m_pitch});

    for (auto& queue : m_subsystemQueues) {
        queue->push_back({drivetrainInGlobal, timestamp, units::radian_t{m_yaw},
                          units::radian_t{m_pitch}, range});
    }

    m_rangeEntry.SetDouble(range.value());
}
