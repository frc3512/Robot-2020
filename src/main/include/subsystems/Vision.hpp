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

class Turret;

/**
 * Vision subsystem.
 */
class Vision : public SubsystemBase {
public:
    // Transformation from camera to drivetrain
    static const frc::Transform2d kCameraInGlobalToTurretInGlobal;

    struct GlobalMeasurement {
        frc::Pose2d drivetrainInGlobal;
        units::second_t timestamp;
    };

    explicit Vision(Turret& turret);

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
     * Subscribe a subsystem to vision data.
     *
     * @param queue Queue to subscribe.
     */
    void SubscribeToVisionData(
        frc3512::static_concurrent_queue<GlobalMeasurement, 8>& queue);

    /**
     * Unsubscribe a subsystem from vision data.
     *
     * @param queue Queue to unsubscribe.
     */
    void UnsubscribeFromVisionData(
        frc3512::static_concurrent_queue<GlobalMeasurement, 8>& queue);

    /**
     * Updates vision sim data with new pose and camera transformation
     *
     * @param drivetrainPose Drivetrain pose to see if target is in range
     * @param turretInGlobalToDrivetrainInGlobal Transformation from the turret
     *                                           coordinate frame to the
     *                                           drivetrain coordinate frame.
     */
    void UpdateVisionMeasurementsSim(
        const frc::Pose2d& drivetrainPose,
        const frc::Transform2d& turretInGlobalToDrivetrainInGlobal);

    void SimulationInit() override;

    void RobotPeriodic() override;

private:
    photonlib::PhotonCamera m_rpiCam{Constants::Vision::kCameraName};

    std::mutex m_subsystemQueuesMutex;

    std::vector<frc3512::static_concurrent_queue<GlobalMeasurement, 8>*>
        m_subsystemQueues;

    Turret& m_turret;

    nt::NetworkTableEntry m_poseEntry = NetworkTableUtil::MakeDoubleArrayEntry(
        "/Diagnostics/Vision/Turret pose");
    nt::NetworkTableEntry m_yawEntry =
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
