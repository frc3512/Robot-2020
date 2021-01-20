// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <photonlib/PhotonCamera.h>

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
        frc::Pose2d pose;
    };

    Vision();
    ~Vision();

    Vision(const Vision&) = delete;
    Vision& operator=(const Vision&) = delete;

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

private:
    nt::NetworkTableEntry m_pose = NetworkTableUtil::MakeDoubleArrayEntry(
        "photonvision/RPI-Cam/targetPose", {0.0, 0.0, 0.0});
    nt::NetworkTableEntry m_latency = NetworkTableUtil::MakeDoubleEntry(
        "photonvision/RPI-Cam/latencyMillis", 0.0);
    NT_EntryListener m_listenerHandle;

    photonlib::PhotonCamera m_rpiCam{"RPI-Cam"};

    frc3512::static_concurrent_queue<GlobalMeasurement, 8> m_measurements;
};

}  // namespace frc3512
