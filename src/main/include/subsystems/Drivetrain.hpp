// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <vector>

#include <Eigen/Core>
#include <adi/ADIS16470_IMU.h>
#include <adi/simulation/ADIS16470_IMUSim.h>
#include <frc/AnalogInput.h>
#include <frc/Encoder.h>
#include <frc/LinearFilter.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/estimator/AngleStatistics.h>
#include <frc/estimator/KalmanFilterLatencyCompensator.h>
#include <frc/estimator/UnscentedKalmanFilter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/simulation/AnalogInputSim.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <networktables/NetworkTableEntry.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>

#include "Constants.hpp"
#include "HWConfig.hpp"
#include "NetworkTableUtil.hpp"
#include "controllers/DrivetrainController.hpp"
#include "controllers/ImplicitModelFollower.hpp"
#include "rev/CANSparkMax.hpp"
#include "static_concurrent_queue.hpp"
#include "subsystems/ControlledSubsystemBase.hpp"
#include "subsystems/Vision.hpp"

namespace frc3512 {

/**
 * The drivetrain subsystem.
 *
 * The drivetrain uses an unscented Kalman filter for state estimation.
 */
class Drivetrain : public ControlledSubsystemBase<7, 2, 5> {
public:
    /// The drivetrain length.
    static constexpr units::meter_t kLength = 0.9398_m;

    /**
     * Distance from middle of robot to intake.
     */
    static constexpr units::meter_t kMiddleOfRobotToIntake = 0.656_m;

    /**
     * Producer-consumer queue for global pose measurements from Vision
     * subsystem.
     */
    static_concurrent_queue<Vision::GlobalMeasurement, 8> visionQueue;

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
     * Returns distance from the left ultrasonic sensor.
     */
    units::meter_t GetLeftUltrasonicDistance() const;

    /**
     * Returns distance from the right ultrasonic sensor.
     */
    units::meter_t GetRightUltrasonicDistance() const;

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
     * constraint with the start and end velocities set to zero.
     */
    static frc::TrajectoryConfig MakeTrajectoryConfig();

    /**
     * Returns a TrajectoryConfig containing a differential drive dynamics
     * constraint and the specified start and end velocities.
     *
     * @param startVelocity The start velocity of the trajectory.
     * @param endVelocity   The end velocity of the trajectory.
     */
    static frc::TrajectoryConfig MakeTrajectoryConfig(
        units::meters_per_second_t startVelocity,
        units::meters_per_second_t endVelocity);

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

    /**
     * Returns how many times the vision measurement was too far from the
     * drivetrain pose estimate.
     */
    int GetPoseMeasurementFaultCounter();

    /**
     * Returns sim pose.
     */
    frc::Pose2d GetSimPose() const;

    void DisabledInit() override;

    void AutonomousInit() override;

    void TeleopInit() override;

    void TestInit() override;

    void RobotPeriodic() override;

    void TeleopPeriodic() override;

    void TestPeriodic() override;

    void ControllerPeriodic() override;

private:
    static const Eigen::Matrix<double, 2, 2> kGlobalR;

    static const frc::LinearSystem<2, 2, 2> kPlant;

    frc::AnalogInput m_leftUltrasonic{
        HWConfig::Drivetrain::kLeftUltrasonicChannel};
    units::meter_t m_leftUltrasonicDistance;
    frc::AnalogInput m_rightUltrasonic{
        HWConfig::Drivetrain::kRightUltrasonicChannel};
    units::meter_t m_rightUltrasonicDistance;
    frc::LinearFilter<units::meter_t> m_leftDistanceFilter =
        frc::LinearFilter<units::meter_t>::SinglePoleIIR(0.1, 0.02_s);

    frc::LinearFilter<units::meter_t> m_rightDistanceFilter =
        frc::LinearFilter<units::meter_t>::SinglePoleIIR(0.1, 0.02_s);

    rev::CANSparkMax m_leftLeader{HWConfig::Drivetrain::kLeftMotorLeaderID,
                                  rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_leftFollower{HWConfig::Drivetrain::kLeftMotorFollowerID,
                                    rev::CANSparkMax::MotorType::kBrushless};
    frc::SpeedControllerGroup m_leftGrbx{m_leftLeader, m_leftFollower};

    rev::CANSparkMax m_rightLeader{HWConfig::Drivetrain::kRightMotorLeaderID,
                                   rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_rightFollower{
        HWConfig::Drivetrain::kRightMotorFollowerID,
        rev::CANSparkMax::MotorType::kBrushless};
    frc::SpeedControllerGroup m_rightGrbx{m_rightLeader, m_rightFollower};

    frc::Encoder m_leftEncoder{HWConfig::Drivetrain::kLeftEncoderA,
                               HWConfig::Drivetrain::kLeftEncoderB};

    frc::Encoder m_rightEncoder{HWConfig::Drivetrain::kRightEncoderA,
                                HWConfig::Drivetrain::kRightEncoderB};

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
        Constants::kControllerPeriod};
    frc::KalmanFilterLatencyCompensator<7, 2, 5,
                                        frc::UnscentedKalmanFilter<7, 2, 5>>
        m_latencyComp;
    DrivetrainController m_controller;
    Eigen::Matrix<double, 2, 1> m_u = Eigen::Matrix<double, 2, 1>::Zero();

    nt::NetworkTableEntry m_leftUltrasonicOutputEntry =
        NetworkTableUtil::MakeDoubleEntry(
            "/Diagnostics/Drivetrain/Outputs/Left Ultrasonic Output");

    nt::NetworkTableEntry m_rightUltrasonicOutputEntry =
        NetworkTableUtil::MakeDoubleEntry(
            "/Diagnostics/Drivetrain/Outputs/Right Ultrasonic Output");

    frc::LinearSystem<2, 2, 2> m_imfRef =
        frc::LinearSystemId::IdentifyDrivetrainSystem(
            DrivetrainController::kLinearV,
            DrivetrainController::kLinearA * 5.0,
            DrivetrainController::kAngularV,
            DrivetrainController::kAngularA * 2.0);
    ImplicitModelFollower<2, 2> m_imf{
        kPlant, m_imfRef, {0.01, 0.01}, {8.0, 8.0}, 20_ms};

    int m_poseMeasurementFaultCounter = 0;
    nt::NetworkTableEntry m_poseMeasurementFaultEntry =
        NetworkTableUtil::MakeDoubleEntry("/Diagnostics/Vision/Vision faults");

    // Simulation variables
    frc::sim::DifferentialDrivetrainSim m_drivetrainSim{
        DrivetrainController::GetPlant(), DrivetrainController::kWidth,
        frc::DCMotor::NEO(2), DrivetrainController::kDriveGearRatio,
        DrivetrainController::kWheelRadius};
    frc::sim::EncoderSim m_leftEncoderSim{m_leftEncoder};
    frc::sim::EncoderSim m_rightEncoderSim{m_rightEncoder};
    frc::sim::ADIS16470_IMUSim m_imuSim{m_imu};
    frc::sim::AnalogInputSim m_leftUltrasonicSim{m_leftUltrasonic};
    frc::sim::AnalogInputSim m_rightUltrasonicSim{m_rightUltrasonic};
    frc::Field2d m_field;

    /**
     * Set drivetrain motors to brake mode, which the feedback controllers
     * expect.
     */
    void SetBrakeMode();

    /**
     * Set drivetrain motors to coast mode so the robot is easier to push when
     * it's disabled.
     */
    void SetCoastMode();
};

}  // namespace frc3512
