// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <photonlib/PhotonCamera.h>
#include <photonlib/SimVisionSystem.h>
#include <units/time.h>

#include "Constants.hpp"
#include "NetworkTableUtil.hpp"
#include "TargetModel.hpp"
#include "static_concurrent_queue.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

/**
 * Vision subsystem.
 */
class Vision : public SubsystemBase {
public:
    // Transformation from camera to drivetrain
    static const frc::Transform2d kCameraInGlobalToTurretInGlobal;

    struct GlobalMeasurement {
        frc::Pose2d targetToTurretInGlobal;
        units::second_t timestamp;
    };

    /**
     * Turns on power to the LED ring light.
     */
    void TurnLEDOn();

    /**
     * Turns off power to the LED ring light.
     */
    void TurnLEDOff();

    /**
     * Returns whether or not the LED ring light is on or off.
     */
    bool IsLEDOn() const;

    /**
     * Allows subsystems to subscribe to vision data
     *
     * @param queue Queue to use to subscribe to vision data
     */
    void SubscribeToVisionData(
        frc3512::static_concurrent_queue<GlobalMeasurement, 8>& queue);

    /**
     * Converts solvePnP data from the networktables into a global turret pose
     * measurement
     */
    void ProcessNewMeasurement();

    /**
     * Updates vision sim data with new pose and camera transformation
     *
     * @param drivetrainPose Drivetrain pose to see if target is in range
     * @param cameraHeading Camera position relative to the drivetrain and turret heading
     */
    void UpdateVisionMeasurementsSim(
        const frc::Pose2d& drivetrainPose,
        const frc::Rotation2d& cameraHeading);

    void SimulationInit() override;

    void RobotPeriodic() override;

private:
    photonlib::PhotonCamera m_rpiCam{Constants::Vision::kCameraName};
    photonlib::PhotonPipelineResult m_result;

    std::mutex m_mutex;

    std::vector<frc3512::static_concurrent_queue<GlobalMeasurement, 8>*>
        m_subsystemQueues;

    nt::NetworkTableEntry m_hasTargetEntry =
        NetworkTableUtil::MakeBoolEntry("/Diagnostics/Vision/Has target");
    nt::NetworkTableEntry m_hasPoseEntry =
        NetworkTableUtil::MakeDoubleArrayEntry("/Diagnostics/Vision/Pose");
    nt::NetworkTableEntry m_hasYawEntry =
        NetworkTableUtil::MakeDoubleEntry("/Diagnostics/Vision/Yaw");

    // Simulation variables
    photonlib::SimVisionSystem m_simVision{
        Constants::Vision::kCameraName,
        Constants::Vision::kCameraDiagonalFOV,
        Constants::Vision::kCameraPitch,
        frc::Transform2d{},
        Constants::Vision::kCameraHeight,
        20_m,
        960,
        720,
        10};
};

}  // namespace frc3512
