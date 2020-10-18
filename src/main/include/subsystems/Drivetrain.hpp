// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>
#include <frc/ADXRS450_Gyro.h>
#include <frc/Encoder.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/simulation/SimDeviceSim.h>
#include <frc2/Timer.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/CANSparkMax.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include "Constants.hpp"
#include "subsystems/ControllerSubsystemBase.hpp"

namespace frc {
class TrajectoryConfig;
}  // namespace frc

namespace frc3512 {

class DrivetrainController;

/**
 * Provides an interface for this year's drive train.
 */
class Drivetrain : public ControllerSubsystemBase {
public:
    static constexpr units::meter_t kLength = 0.9398_m;

    Drivetrain();
    virtual ~Drivetrain();

    Drivetrain(Drivetrain&&) = default;
    Drivetrain& operator=(Drivetrain&&) = default;

    /**
     * Curvature drive method for differential drive platform.
     *
     * The rotation argument controls the curvature of the robot's path rather
     * than its rate of heading change. This makes the robot more controllable
     * at high speeds. Constant-curvature turning can be overridden for
     * turn-in-place maneuvers.
     *
     * @param xSpeed           The robot's speed along the X axis [-1.0..1.0].
     *                         Forward is positive.
     * @param zRotation        The robot's rotation rate around the Z axis
     *                         [-1.0..1.0]. Clockwise is positive.
     * @param allowTurnInPlace If set, overrides constant-curvature turning for
     *                         turn-in-place maneuvers.
     */
    void Drive(double xSpeed, double zRotation, bool allowTurnInPlace = false);

    /**
     * Returns the drivetrain's pose estimate.
     */
    frc::Pose2d GetPose() const;

    /**
     * Returns gyro angle.
     *
     * @return angle in degrees
     */
    units::radian_t GetAngle() const;

    /**
     * Returns gyro angular rate.
     *
     * @return angular rate in degrees per second
     */
    units::radians_per_second_t GetAngularRate() const;

    /**
     * Returns left encoder displacement.
     *
     * @return left displacement
     */
    units::meter_t GetLeftPosition() const;

    /**
     * Returns right encoder displacement.
     *
     * @return right displacement
     */
    units::meter_t GetRightPosition() const;

    /**
     * Returns right encoder displacement.
     *
     * @return left rate
     */
    units::meters_per_second_t GetLeftVelocity() const;

    /**
     * Returns right encoder displacement.
     *
     * @return right rate
     */
    units::meters_per_second_t GetRightVelocity() const;

    /**
     * Resets all sensors and controller.
     */
    void Reset(const frc::Pose2d& initialPose = frc::Pose2d());

    /**
     * Enables the controller.
     */
    void EnableController();

    /**
     * Disables the controller.
     */
    void DisableController();

    /**
     * Returns true if controller is enabled and false if controller is
     * disabled.
     *
     * @return whether or not the controller is enabled
     */
    bool IsControllerEnabled() const;

    /**
     * Set global measurements.
     *
     * @param x         X position of the robot in meters.
     * @param y         Y position of the robot in meters.
     * @param timestamp Absolute time the translation data comes from.
     */
    void CorrectWithGlobalOutputs(units::meter_t x, units::meter_t y,
                                  int64_t timestamp);

    /**
     * Sets the waypoints for a generated trajectory.
     *
     * @param start    Starting pose.
     * @param interior Intermediate waypoints excluding heading.
     * @param end      Ending pose.
     */
    void SetWaypoints(const frc::Pose2d& start,
                      const std::vector<frc::Translation2d>& interior,
                      const frc::Pose2d& end);

    /**
     * Sets the waypoints for a generated trajectory.
     *
     * @param start    Starting pose.
     * @param interior Intermediate waypoints excluding heading.
     * @param end      Ending pose.
     * @param config   TrajectoryConfig for this trajectory. This can include
     *                 constraints on the trajectory dynamics. If adding custom
     *                 constraints, it is recommended to start with the config
     *                 returned by MakeTrajectoryConfig() so differential drive
     *                 dynamics constraints are included automatically.
     */
    void SetWaypoints(const frc::Pose2d& start,
                      const std::vector<frc::Translation2d>& interior,
                      const frc::Pose2d& end, frc::TrajectoryConfig& config);

    /**
     * Returns whether the drivetrain controller is at the goal waypoint.
     */
    bool AtGoal() const;

    Eigen::Matrix<double, 10, 1> GetNextXhat() const;

    void DisabledInit() override;

    void AutonomousInit() override;

    void TeleopInit() override;

    void RobotPeriodic() override;

    void TeleopPeriodic() override;

protected:
    void ControllerPeriodic() override;

private:
    rev::CANSparkMax m_leftMaster{Constants::Drivetrain::kLeftMasterPort,
                                  rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_leftSlave{Constants::Drivetrain::kLeftSlavePort,
                                 rev::CANSparkMax::MotorType::kBrushless};
    frc::SpeedControllerGroup m_leftGrbx{m_leftMaster, m_leftSlave};

    rev::CANSparkMax m_rightMaster{Constants::Drivetrain::kRightMasterPort,
                                   rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_rightSlave{Constants::Drivetrain::kRightSlavePort,
                                  rev::CANSparkMax::MotorType::kBrushless};
    frc::SpeedControllerGroup m_rightGrbx{m_rightMaster, m_rightSlave};

    frc::Encoder m_leftEncoder{Constants::Drivetrain::kLeftEncoderA,
                               Constants::Drivetrain::kLeftEncoderB};

    frc::Encoder m_rightEncoder{Constants::Drivetrain::kRightEncoderA,
                                Constants::Drivetrain::kRightEncoderB};

    frc::DifferentialDrive m_drive{m_leftGrbx, m_rightGrbx};

    frc::ADXRS450_Gyro m_gyro;
    units::radian_t m_headingOffset = 0_rad;

    // Controller
    std::unique_ptr<DrivetrainController> m_controller;

    nt::NetworkTableInstance m_inst = nt::NetworkTableInstance::GetDefault();
    nt::NetworkTableEntry m_leftEncoderEntry =
        m_inst.GetEntry("Diagnostics/Drivetrain/Left encoder");
    nt::NetworkTableEntry m_rightEncoderEntry =
        m_inst.GetEntry("Diagnostics/Drivetrain/Right encoder");
    nt::NetworkTableEntry m_headingEntry =
        m_inst.GetEntry("Diagnostics/Drivetrain/Heading");

    // Simulation variables
    frc::sim::DifferentialDrivetrainSim m_drivetrainSim;
    frc::sim::EncoderSim m_leftEncoderSim{m_leftEncoder};
    frc::sim::EncoderSim m_rightEncoderSim{m_rightEncoder};
    hal::SimDouble m_angleSim;
};

}  // namespace frc3512
