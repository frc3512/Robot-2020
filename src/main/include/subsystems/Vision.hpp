// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <photonlib/PhotonCamera.h>
#include <photonlib/SimVisionSystem.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
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
        units::second_t timestamp;
        units::radian_t pitch;
        units::radian_t yaw;
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
     * Returns the most recent global measurement
     */
    std::optional<GlobalMeasurement> GetGlobalMeasurement();

    /**
     * Converts solvePnP data from the networktables into a global turret pose
     * measurement
     */
    void ProcessNewMeasurement();

    /**
     * Updates vision sim data with new pose and camera transformation
     *
     * @param cameraToRobot new camera to robot transform as turret moves
     * @param drivetrainPose drivetrain pose to see if target is in range
     */
    void UpdateVisionMeasurementsSim(const frc::Transform2d& cameraToRobot,
                                     const frc::Pose2d& drivetrainPose);

    void SimulationInit() override;

    void RobotPeriodic() override;

private:
    photonlib::PhotonCamera m_rpiCam{"Gloworm"};
    photonlib::PhotonPipelineResult m_result;

    frc3512::static_concurrent_queue<GlobalMeasurement, 8> m_measurements;

    // Simulation variables
    photonlib::SimVisionSystem m_simVision{
        "Glowworm",
        Constants::Vision::kCameraDiagonalFOV,
        Constants::Vision::kCameraPitch,
        kCameraInGlobalToTurretInGlobal,
        Constants::Vision::kCameraHeight,
        20_m,
        640,
        480,
        10};
};

}  // namespace frc3512
