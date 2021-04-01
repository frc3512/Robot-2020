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

std::optional<Vision::GlobalMeasurement> Vision::GetGlobalMeasurement() {
    return m_measurements.pop();
}

void Vision::ProcessNewMeasurement() {
    units::second_t latency = m_result.GetLatency();

    photonlib::PhotonTrackedTarget target = m_result.GetBestTarget();

    auto timestamp = frc2::Timer::GetFPGATimestamp();
    timestamp -= latency;

    m_measurements.push({timestamp, units::degree_t{target.GetPitch()},
                            units::degree_t{target.GetYaw()}});
}

void Vision::UpdateVisionMeasurementsSim(const frc::Transform2d& cameraToRobot,
                                         const frc::Pose2d& drivetrainPose) {
    m_simVision.MoveCamera(cameraToRobot, Constants::Vision::kCameraHeight,
                           Constants::Vision::kCameraPitch);
    m_simVision.ProcessFrame(drivetrainPose);
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
    photonlib::PhotonPipelineResult m_result = m_rpiCam.GetLatestResult();

    if (m_result.HasTargets() &&
        !frc::DriverStation::GetInstance().IsDisabled()) {
        ProcessNewMeasurement();
    }
}
