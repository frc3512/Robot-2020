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

    // PnP data is transformation from camera's coordinate frame to target's
    // coordinate frame
    photonlib::PhotonTrackedTarget target = m_result.GetBestTarget();

    // If the target is empty then everything will be zero
    if (target.GetPitch() != 0) {
        auto timestamp = frc2::Timer::GetFPGATimestamp();
        timestamp -= latency;

        m_measurements.push({timestamp, units::degree_t{target.GetPitch()},
                             units::degree_t{target.GetYaw()}});
    }
}

void Vision::RobotPeriodic() {
    photonlib::PhotonPipelineResult m_result = m_rpiCam.GetLatestResult();

    if (m_result.HasTargets()) {
        ProcessNewMeasurement();
    }
}
