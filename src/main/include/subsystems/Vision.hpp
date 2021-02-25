// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

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
    ~Vision() override;

    Vision(const Vision&) = delete;
    Vision& operator=(const Vision&) = delete;

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
    nt::NetworkTableEntry m_pose =
        NetworkTableUtil::MakeDoubleArrayEntry("/Vision/Pose", {0.0, 0.0, 0.0});
    nt::NetworkTableEntry m_latency =
        NetworkTableUtil::MakeDoubleEntry("/Vision/Timestamp", 0.0);
    NT_EntryListener m_listenerHandle;

    frc3512::static_concurrent_queue<GlobalMeasurement, 8> m_measurements;
};

}  // namespace frc3512
