// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <ntcore_c.h>

#include <memory>
#include <optional>

#include <frc/geometry/Translation2d.h>
#include <frc/logging/CSVLogFile.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <units/units.h>
#include <wpi/math>
#include <wpi/mutex.h>
#include <wpi/static_circular_buffer.h>

#include "controllers/DrivetrainController.hpp"
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

    ~Vision() override = default;

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
    nt::NetworkTableInstance m_inst;
    std::shared_ptr<nt::NetworkTable> m_ledTable;
    nt::NetworkTableEntry m_ledIsOn;
    std::shared_ptr<nt::NetworkTable> m_poseTable;
    std::shared_ptr<nt::NetworkTable> m_rpiTable;
    nt::NetworkTableEntry m_pose;
    nt::NetworkTableEntry m_latency;

    wpi::mutex m_measurementMutex;
    wpi::static_circular_buffer<GlobalMeasurement, 8> m_measurements;
};
}  // namespace frc3512
