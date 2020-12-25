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
#include <frc/simulation/ADXRS450_GyroSim.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc2/Timer.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include "Constants.hpp"
#include "rev/CANSparkMax.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

class DrivetrainController;

/**
 * Drivetrain subsystem.
 */
class Drivetrain : public SubsystemBase {
public:
    static constexpr units::meter_t kLength = 0.9398_m;
    static constexpr units::meter_t kMiddleOfRobotToIntake = 0.656_m;

    Drivetrain();
    ~Drivetrain();

    Drivetrain(const Drivetrain&) = delete;
    Drivetrain& operator=(const Drivetrain&) = delete;

    /**
     * Returns the drivetrain's pose estimate.
     */
    frc::Pose2d GetPose() const;

    /**
     * Returns gyro's heading measurement in the global coordinate frame.
     */
    units::radian_t GetAngle() const;

    /**
     * Returns gyro angular rate.
     */
    units::radians_per_second_t GetAngularRate() const;

    /**
     * Returns left encoder displacement.
     */
    units::meter_t GetLeftPosition() const;

    /**
     * Returns right encoder displacement.
     */
    units::meter_t GetRightPosition() const;

    /**
     * Returns left encoder velocity.
     */
    units::meters_per_second_t GetLeftVelocity() const;

    /**
     * Returns right encoder velocity.
     */
    units::meters_per_second_t GetRightVelocity() const;

    /**
     * Resets all sensors and controller.
     */
    void Reset(const frc::Pose2d& initialPose = frc::Pose2d());

    /**
     * Set global measurements.
     *
     * @param x         X position of the robot in meters.
     * @param y         Y position of the robot in meters.
     * @param timestamp Absolute time the translation data comes from.
     */
    void CorrectWithGlobalOutputs(units::meter_t x, units::meter_t y,
                                  units::second_t timestamp);

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
                      const frc::Pose2d& end,
                      const frc::TrajectoryConfig& config);

    /**
     * Sets the waypoints for a generated trajectory.
     *
     * @param waypoints Waypoints.
     */
    void SetWaypoints(const std::vector<frc::Pose2d>& waypoints);

    /**
     * Sets the waypoints for a generated trajectory.
     *
     * @param waypoints Waypoints.
     * @param config    TrajectoryConfig for this trajectory. This can include
     *                  constraints on the trajectory dynamics. If adding custom
     *                  constraints, it is recommended to start with the config
     *                  returned by MakeTrajectoryConfig() so differential drive
     *                  dynamics constraints are included automatically.
     */
    void SetWaypoints(const std::vector<frc::Pose2d>& waypoints,
                      const frc::TrajectoryConfig& config);

    /**
     * Returns a TrajectoryConfig containing a differential drive dynamics
     * constraint.
     */
    static frc::TrajectoryConfig MakeTrajectoryConfig();

    /**
     * Returns whether the drivetrain controller is at the goal waypoint.
     */
    bool AtGoal() const;

    /**
     * Returns the drivetrain state estimate.
     */
    Eigen::Matrix<double, 7, 1> GetStates() const;

    /**
     * Returns current drawn in simulation.
     */
    units::ampere_t GetCurrentDraw() const;

    void DisabledInit() override;

    void AutonomousInit() override;

    void TeleopInit() override;

    void RobotPeriodic() override;

    void TeleopPeriodic() override;

    void ControllerPeriodic();

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

    std::unique_ptr<DrivetrainController> m_controller;

    nt::NetworkTableInstance m_inst = nt::NetworkTableInstance::GetDefault();
    nt::NetworkTableEntry m_xStateEntry =
        m_inst.GetEntry("/Diagnostics/Drivetrain/States/X");
    nt::NetworkTableEntry m_yStateEntry =
        m_inst.GetEntry("/Diagnostics/Drivetrain/States/Y");
    nt::NetworkTableEntry m_headingStateEntry =
        m_inst.GetEntry("/Diagnostics/Drivetrain/States/Heading");
    nt::NetworkTableEntry m_leftVelocityStateEntry =
        m_inst.GetEntry("/Diagnostics/Drivetrain/States/Left velocity");
    nt::NetworkTableEntry m_rightVelocityStateEntry =
        m_inst.GetEntry("/Diagnostics/Drivetrain/States/Right velocity");
    nt::NetworkTableEntry m_leftPositionStateEntry =
        m_inst.GetEntry("/Diagnostics/Drivetrain/States/Left position");
    nt::NetworkTableEntry m_rightPositionStateEntry =
        m_inst.GetEntry("/Diagnostics/Drivetrain/States/Right position");
    nt::NetworkTableEntry m_leftVoltageInputEntry =
        m_inst.GetEntry("/Diagnostics/Drivetrain/Inputs/Left voltage");
    nt::NetworkTableEntry m_rightVoltageInputEntry =
        m_inst.GetEntry("/Diagnostics/Drivetrain/Inputs/Right voltage");
    nt::NetworkTableEntry m_headingOutputEntry =
        m_inst.GetEntry("/Diagnostics/Drivetrain/Outputs/Heading");
    nt::NetworkTableEntry m_leftPositionOutputEntry =
        m_inst.GetEntry("/Diagnostics/Drivetrain/Outputs/Left position");
    nt::NetworkTableEntry m_rightPositionOutputEntry =
        m_inst.GetEntry("/Diagnostics/Drivetrain/Outputs/Right position");

    // Simulation variables
    frc::sim::DifferentialDrivetrainSim m_drivetrainSim;
    frc::sim::EncoderSim m_leftEncoderSim{m_leftEncoder};
    frc::sim::EncoderSim m_rightEncoderSim{m_rightEncoder};
    frc::sim::ADXRS450_GyroSim m_gyroSim{m_gyro};
    frc::Field2d m_field;
};

}  // namespace frc3512
