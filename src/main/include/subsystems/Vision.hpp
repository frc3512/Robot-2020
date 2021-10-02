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
 * The vision subsystem.
 */
class Vision : public SubsystemBase {
public:
    /// Camera name in NetworkTables
    static constexpr char kCameraName[] = "gloworm";

    /// Camera height
    static constexpr units::meter_t kCameraHeight = 39_in;

    /// Camera pitch
    static constexpr units::degree_t kCameraPitch = 22.8_deg;

    /// Pi camera V1 diagonal field of view
    static constexpr units::degree_t kCameraDiagonalFOV = 74.8_deg;

    /// Transformation from camera to drivetrain
    static const frc::Transform2d kCameraInGlobalToTurretInGlobal;

    /**
     * Container for global measurements.
     *
     * These are sent to the drivetrain subsystem via a producer-consumer queue.
     */
    struct GlobalMeasurement {
        /// Drivetrain global pose measurement
        frc::Pose2d drivetrainInGlobal;
        /// Timestamp at which the measurement was taken
        units::second_t timestamp;
        /// Yaw reported by photonvision
        units::radian_t yaw;
        /// Pitch reported by photonvision
        units::radian_t pitch;
        /// Range from robot to target
        units::meter_t range;
    };

    /**
     * Constructs a Vision.
     *
     * @param turret Turret subsystem.
     */
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

    /**
     * Returns whether or not a vision target is within the camera's field of
     * view.
     */
    bool IsTargetDetected() const;

    void SimulationInit() override;

    void RobotPeriodic() override;

private:
    photonlib::PhotonCamera m_rpiCam{kCameraName};
    units::degree_t m_pitch;
    units::degree_t m_yaw;

    bool m_isTargetDetected = false;

    std::mutex m_subsystemQueuesMutex;

    std::vector<frc3512::static_concurrent_queue<GlobalMeasurement, 8>*>
        m_subsystemQueues;

    Turret& m_turret;

    nt::NetworkTableEntry m_poseEntry = NetworkTableUtil::MakeDoubleArrayEntry(
        "/Diagnostics/Vision/Turret pose");
    nt::NetworkTableEntry m_yawEntry =
        NetworkTableUtil::MakeDoubleEntry("/Diagnostics/Vision/Yaw");
    nt::NetworkTableEntry m_rangeEntry =
        NetworkTableUtil::MakeDoubleEntry("/Diagnostics/Vision/Range Estimate");

    // Simulation variables
    photonlib::SimVisionSystem m_simVision{kCameraName,
                                           kCameraDiagonalFOV,
                                           kCameraPitch,
                                           frc::Transform2d{},
                                           kCameraHeight,
                                           20_m,
                                           960,
                                           720,
                                           10};
};

}  // namespace frc3512
