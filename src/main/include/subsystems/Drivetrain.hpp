// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <vector>

#include <Eigen/Core>
#include <adi/ADIS16470_IMU.h>
#include <adi/simulation/ADIS16470_IMUSim.h>
#include <frc/Encoder.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/estimator/AngleStatistics.h>
#include <frc/estimator/KalmanFilterLatencyCompensator.h>
#include <frc/estimator/UnscentedKalmanFilter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include "Constants.hpp"
#include "NetworkTableUtil.hpp"
#include "controllers/DrivetrainController.hpp"
#include "controllers/ImplicitModelFollower.hpp"
#include "rev/CANSparkMax.hpp"
#include "subsystems/ControlledSubsystemBase.hpp"

namespace frc3512 {

/**
 * Drivetrain subsystem.
 */
class Drivetrain : public ControlledSubsystemBase<7, 2, 5> {
public:
    static constexpr units::meter_t kLength = 0.9398_m;
    static constexpr units::meter_t kMiddleOfRobotToIntake = 0.656_m;

    Drivetrain();

    Drivetrain(const Drivetrain&) = delete;
    Drivetrain& operator=(const Drivetrain&) = delete;

    /**
     * Returns the drivetrain's reference pose.
     */
    frc::Pose2d GetReferencePose() const;

    /**
     * Returns the drivetrain's pose estimate.
     */
    frc::Pose2d GetPose() const;

    /**
     * Returns gyro's heading measurement in the global coordinate frame.
     */
    units::radian_t GetAngle() const;

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
     * Returns longitudinal acceleration from IMU.
     */
    units::meters_per_second_squared_t GetAccelerationX() const;

    /**
     * Returns lateral acceleration from IMU.
     */
    units::meters_per_second_squared_t GetAccelerationY() const;

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
     * Adds a trajectory with the given waypoints.
     *
     * This can be called more than once to create a queue of trajectories.
     * Closed-loop control will be enabled to track the first trajectory.
     *
     * @param start    Starting pose.
     * @param interior Intermediate waypoints excluding heading.
     * @param end      Ending pose.
     */
    void AddTrajectory(const frc::Pose2d& start,
                       const std::vector<frc::Translation2d>& interior,
                       const frc::Pose2d& end);

    /**
     * Adds a trajectory with the given waypoints.
     *
     * This can be called more than once to create a queue of trajectories.
     * Closed-loop control will be enabled to track the first trajectory.
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
    void AddTrajectory(const frc::Pose2d& start,
                       const std::vector<frc::Translation2d>& interior,
                       const frc::Pose2d& end,
                       const frc::TrajectoryConfig& config);

    /**
     * Adds a trajectory with the given waypoints.
     *
     * This can be called more than once to create a queue of trajectories.
     * Closed-loop control will be enabled to track the first trajectory.
     *
     * @param waypoints Waypoints.
     */
    void AddTrajectory(const std::vector<frc::Pose2d>& waypoints);

    /**
     * Adds a trajectory with the given waypoints.
     *
     * This can be called more than once to create a queue of trajectories.
     * Closed-loop control will be enabled to track the first trajectory.
     *
     * @param waypoints Waypoints.
     * @param config    TrajectoryConfig for this trajectory. This can include
     *                  constraints on the trajectory dynamics. If adding custom
     *                  constraints, it is recommended to start with the config
     *                  returned by MakeTrajectoryConfig() so differential drive
     *                  dynamics constraints are included automatically.
     */
    void AddTrajectory(const std::vector<frc::Pose2d>& waypoints,
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
    const Eigen::Matrix<double, 7, 1>& GetStates() const;

    /**
     * Returns the drivetrain inputs.
     */
    const Eigen::Matrix<double, 2, 1>& GetInputs() const;

    /**
     * Returns current drawn in simulation.
     */
    units::ampere_t GetCurrentDraw() const;

    void DisabledInit() override { Disable(); }

    void AutonomousInit() override { Enable(); }

    void TeleopInit() override;

    void RobotPeriodic() override;

    void TeleopPeriodic() override;

    void ControllerPeriodic() override;

private:
    static const Eigen::Matrix<double, 2, 2> kGlobalR;

    static const frc::LinearSystem<2, 2, 2> kPlant;

    rev::CANSparkMax m_leftLeader{Constants::Drivetrain::kLeftLeaderPort,
                                  rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_leftFollower{Constants::Drivetrain::kLeftFollowerPort,
                                    rev::CANSparkMax::MotorType::kBrushless};
    frc::SpeedControllerGroup m_leftGrbx{m_leftLeader, m_leftFollower};

    rev::CANSparkMax m_rightLeader{Constants::Drivetrain::kRightLeaderPort,
                                   rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_rightFollower{Constants::Drivetrain::kRightFollowerPort,
                                     rev::CANSparkMax::MotorType::kBrushless};
    frc::SpeedControllerGroup m_rightGrbx{m_rightLeader, m_rightFollower};

    frc::Encoder m_leftEncoder{Constants::Drivetrain::kLeftEncoderA,
                               Constants::Drivetrain::kLeftEncoderB};

    frc::Encoder m_rightEncoder{Constants::Drivetrain::kRightEncoderA,
                                Constants::Drivetrain::kRightEncoderB};

    frc::ADIS16470_IMU m_imu;
    units::radian_t m_headingOffset = 0_rad;

    frc::UnscentedKalmanFilter<7, 2, 5> m_observer{
        DrivetrainController::Dynamics,
        DrivetrainController::LocalMeasurementModel,
        {0.002, 0.002, 0.0001, 1.5, 1.5, 0.5, 0.5},
        {0.0001, 0.005, 0.005, 7.0, 7.0},
        frc::AngleMean<7, 7>(2),
        frc::AngleMean<5, 7>(0),
        frc::AngleResidual<7>(2),
        frc::AngleResidual<5>(0),
        frc::AngleAdd<7>(2),
        Constants::kDt};
    frc::KalmanFilterLatencyCompensator<7, 2, 5,
                                        frc::UnscentedKalmanFilter<7, 2, 5>>
        m_latencyComp;
    DrivetrainController m_controller;
    Eigen::Matrix<double, 2, 1> m_u = Eigen::Matrix<double, 2, 1>::Zero();

    frc::LinearSystem<2, 2, 2> m_imfRef =
        frc::LinearSystemId::IdentifyDrivetrainSystem(
            DrivetrainController::kLinearV,
            DrivetrainController::kLinearA * 2.0,
            DrivetrainController::kAngularV,
            DrivetrainController::kAngularA * 2.0);
    ImplicitModelFollower<2, 2> m_imf{
        kPlant, m_imfRef, {0.01, 0.01}, {12.0, 12.0}, 20_ms};

    nt::NetworkTableEntry m_xStateEntry = NetworkTableUtil::MakeDoubleEntry(
        "/Diagnostics/Drivetrain/States/X", 0.0);
    nt::NetworkTableEntry m_yStateEntry = NetworkTableUtil::MakeDoubleEntry(
        "/Diagnostics/Drivetrain/States/Y", 0.0);
    nt::NetworkTableEntry m_headingStateEntry =
        NetworkTableUtil::MakeDoubleEntry(
            "/Diagnostics/Drivetrain/States/Heading", 0.0);
    nt::NetworkTableEntry m_leftVelocityStateEntry =
        NetworkTableUtil::MakeDoubleEntry(
            "/Diagnostics/Drivetrain/States/Left velocity", 0.0);
    nt::NetworkTableEntry m_rightVelocityStateEntry =
        NetworkTableUtil::MakeDoubleEntry(
            "/Diagnostics/Drivetrain/States/Right velocity", 0.0);
    nt::NetworkTableEntry m_leftPositionStateEntry =
        NetworkTableUtil::MakeDoubleEntry(
            "/Diagnostics/Drivetrain/States/Left position", 0.0);
    nt::NetworkTableEntry m_rightPositionStateEntry =
        NetworkTableUtil::MakeDoubleEntry(
            "/Diagnostics/Drivetrain/States/Right position", 0.0);
    nt::NetworkTableEntry m_leftVoltageInputEntry =
        NetworkTableUtil::MakeDoubleEntry(
            "/Diagnostics/Drivetrain/Inputs/Left voltage", 0.0);
    nt::NetworkTableEntry m_rightVoltageInputEntry =
        NetworkTableUtil::MakeDoubleEntry(
            "/Diagnostics/Drivetrain/Inputs/Right voltage", 0.0);
    nt::NetworkTableEntry m_headingOutputEntry =
        NetworkTableUtil::MakeDoubleEntry(
            "/Diagnostics/Drivetrain/Outputs/Heading", 0.0);
    nt::NetworkTableEntry m_leftPositionOutputEntry =
        NetworkTableUtil::MakeDoubleEntry(
            "/Diagnostics/Drivetrain/Outputs/Left position", 0.0);
    nt::NetworkTableEntry m_rightPositionOutputEntry =
        NetworkTableUtil::MakeDoubleEntry(
            "/Diagnostics/Drivetrain/Outputs/Right position", 0.0);
    nt::NetworkTableEntry m_accelerationXOutputEntry =
        NetworkTableUtil::MakeDoubleEntry(
            "/Diagnostics/Drivetrain/Outputs/AccelerationX", 0.0);
    nt::NetworkTableEntry m_accelerationYOutputEntry =
        NetworkTableUtil::MakeDoubleEntry(
            "/Diagnostics/Drivetrain/Outputs/AccelerationY", 0.0);

    // Simulation variables
    frc::sim::DifferentialDrivetrainSim m_drivetrainSim{
        DrivetrainController::GetPlant(), DrivetrainController::kWidth,
        frc::DCMotor::NEO(2), DrivetrainController::kDriveGearRatio,
        DrivetrainController::kWheelRadius};
    frc::sim::EncoderSim m_leftEncoderSim{m_leftEncoder};
    frc::sim::EncoderSim m_rightEncoderSim{m_rightEncoder};
    frc::sim::ADIS16470_IMUSim m_imuSim{m_imu};
    frc::Field2d m_field;
};

}  // namespace frc3512
