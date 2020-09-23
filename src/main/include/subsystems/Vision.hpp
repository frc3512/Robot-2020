// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/logging/CSVLogFile.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <wpi/math>

#include "controllers/DrivetrainController.hpp"
#include "static_concurrent_queue.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

class Vision : public SubsystemBase {
public:
    // Transformation from camera to drivetrain
    static const frc::Transform2d kCameraInGlobalToTurretInGlobal;

    struct GlobalMeasurement {
        int64_t timestamp;
        frc::Pose2d pose;
    };

    Vision();
    ~Vision();
    Vision(Vision&&) = default;
    Vision& operator=(Vision&&) = default;

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
    nt::NetworkTableInstance m_inst = nt::NetworkTableInstance::GetDefault();
    nt::NetworkTableEntry m_ledIsOn =
        m_inst.GetEntry("LED Ring Light/LED-State");
    nt::NetworkTableEntry m_pose =
        m_inst.GetEntry("chameleon-vision/RPI-Cam/target-Pose");
    nt::NetworkTableEntry m_latency =
        m_inst.GetEntry("chameleon-vision/RPI-Cam/latency");
    NT_EntryListener m_listenerHandle;

    frc3512::static_concurrent_queue<GlobalMeasurement, 8> m_measurements;
};

}  // namespace frc3512
