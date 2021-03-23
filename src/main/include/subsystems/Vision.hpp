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

#include "NetworkTableUtil.hpp"
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

    void RobotPeriodic() override;

private:
    photonlib::PhotonCamera m_rpiCam{"RPI-Cam"};
    photonlib::PhotonPipelineResult m_result;

    frc3512::static_concurrent_queue<GlobalMeasurement, 8> m_measurements;
};

}  // namespace frc3512
