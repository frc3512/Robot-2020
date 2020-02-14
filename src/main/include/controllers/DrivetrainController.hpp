// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <array>
#include <chrono>
#include <functional>
#include <tuple>
#include <vector>

#include <Eigen/Core>
#include <frc/estimator/ExtendedKalmanFilter.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/logging/CSVLogFile.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/trajectory/Trajectory.h>
#include <units/units.h>
#include <wpi/math>
#include <wpi/mutex.h>

#include "Constants.hpp"

namespace frc3512 {

class DrivetrainController {
public:
    class State {
    public:
        static constexpr int kX = 0;
        static constexpr int kY = 1;
        static constexpr int kHeading = 2;
        static constexpr int kLeftVelocity = 3;
        static constexpr int kRightVelocity = 4;
        static constexpr int kLeftPosition = 5;
        static constexpr int kRightPosition = 6;
        static constexpr int kLeftVoltageError = 7;
        static constexpr int kRightVoltageError = 8;
        static constexpr int kAngularVelocityError = 9;
    };

    class Input {
    public:
        static constexpr int kLeftVoltage = 0;
        static constexpr int kRightVoltage = 1;
    };

    class LocalOutput {
    public:
        static constexpr int kHeading = 0;
        static constexpr int kLeftPosition = 1;
        static constexpr int kRightPosition = 2;
    };

    class GlobalOutput {
    public:
        static constexpr int kX = 0;
        static constexpr int kY = 1;
        static constexpr int kHeading = 2;
        static constexpr int kLeftPosition = 3;
        static constexpr int kRightPosition = 4;
        static constexpr int kAngularVelocity = 5;
    };

    /**
     * Constructs a drivetrain controller with the given coefficients.
     *
     * @param Qelems The maximum desired error tolerance for each state.
     * @param Relems The maximum desired control effort for each input.
     * @param dt     Discretization timestep.
     */
    DrivetrainController(const std::array<double, 5>& Qelems,
                         const std::array<double, 2>& Relems,
                         units::second_t dt);

    DrivetrainController(const DrivetrainController&) = delete;
    DrivetrainController& operator=(const DrivetrainController&) = delete;

    void Enable();
    void Disable();
    bool IsEnabled() const;

    void SetWaypoints(const std::vector<frc::Pose2d>& waypoints);

    /**
     * Returns whether the drivetrain controller is at the goal waypoint.
     */
    bool AtGoal();

    /**
     * Set local measurements.
     *
     * @param heading       Angle of the robot.
     * @param leftPosition  Encoder count of left side in meters.
     * @param rightPosition Encoder count of right side in meters.
     */
    void SetMeasuredLocalOutputs(units::radian_t heading,
                                 units::meter_t leftPosition,
                                 units::meter_t rightPosition);

    /**
     * Set global measurements.
     *
     * @param x             X position of the robot in meters.
     * @param y             Y position of the robot in meters.
     * @param heading       Angle of the robot.
     * @param leftPosition  Encoder count of left side in meters.
     * @param rightPosition Encoder count of right side in meters.
     * @param angularVelocity Angular velocity of the robot in radians per
     * second.
     */
    void SetMeasuredGlobalOutputs(units::meter_t x, units::meter_t y,
                                  units::radian_t heading,
                                  units::meter_t leftPosition,
                                  units::meter_t rightPosition,
                                  units::radians_per_second_t angularVelocity);

    /**
     * Returns the current references.
     *
     * x, y, heading, left velocity, and right velocity.
     */
    const Eigen::Matrix<double, 5, 1>& GetReferences() const;

    /**
     * Returns the current state estimate.
     *
     * x, y, heading, left position, left velocity, right position,
     * right velocity, left voltage error, right voltage error, and angle error.
     */
    const Eigen::Matrix<double, 10, 1>& GetStates() const;

    /**
     * Returns the control inputs.
     *
     * left voltage and right voltage.
     */
    Eigen::Matrix<double, 2, 1> GetInputs() const;

    /**
     * Returns the currently set local outputs.
     *
     * heading, left position, left velocity, right position,
     * right velocity, and angular velocity.
     */
    const Eigen::Matrix<double, 3, 1>& GetOutputs() const;

    /**
     * Returns the estimated outputs based on the current state estimate.
     *
     * This provides only local measurements.
     */
    Eigen::Matrix<double, 3, 1> EstimatedLocalOutputs() const;

    /**
     * Returns the estimated outputs based on the current state estimate.
     *
     * This provides global measurements (including pose).
     */
    Eigen::Matrix<double, 6, 1> EstimatedGlobalOutputs() const;

    /**
     * Executes the control loop for a cycle.
     *
     * @param dt Timestep between each Update() call
     */
    void Update(units::second_t dt, units::second_t elapsedTime);

    /**
     * Resets any internal state.
     */
    void Reset();

    /**
     * Resets any internal state.
     *
     * @param initialPose Initial pose for state estimate.
     */
    void Reset(const frc::Pose2d& initialPose);

    Eigen::Matrix<double, 2, 1> Controller(
        const Eigen::Matrix<double, 10, 1>& x,
        const Eigen::Matrix<double, 5, 1>& r);

    static Eigen::Matrix<double, 10, 1> Dynamics(
        const Eigen::Matrix<double, 10, 1>& x,
        const Eigen::Matrix<double, 2, 1>& u);

    static Eigen::Matrix<double, 3, 1> LocalMeasurementModel(
        const Eigen::Matrix<double, 10, 1>& x,
        const Eigen::Matrix<double, 2, 1>& u);

    static Eigen::Matrix<double, 6, 1> GlobalMeasurementModel(
        const Eigen::Matrix<double, 10, 1>& x,
        const Eigen::Matrix<double, 2, 1>& u);

private:
    // Robot radius
    static constexpr auto rb = Constants::Drivetrain::kWidth / 2.0;

    static frc::LinearSystem<2, 2, 2> m_plant;

    // The current sensor measurements
    Eigen::Matrix<double, 3, 1> m_localY;
    Eigen::Matrix<double, 6, 1> m_globalY;

    // Design observer
    // States: [x position, y position, heading,
    //          left velocity, right velocity,
    //          left position, right position,
    //          left voltage error, right voltage error, angle error]
    //
    // Inputs: [left voltage, right voltage]
    //
    // Outputs (local): [left position, right position,
    //                   angular velocity]
    //
    // Outputs (global): [x position, y position, heading,
    //                    left position, right position,
    //                    angular velocity]
    frc::ExtendedKalmanFilter<10, 2, 3> m_observer{
        Dynamics,
        LocalMeasurementModel,
        {0.002, 0.002, 0.0001, 1.5, 1.5, 0.5, 0.5, 10.0, 10.0, 2.0},
        {0.0001, 0.005, 0.005},
        Constants::kDt};

    // XXX: For testing only. This is used to verify the EKF pose because
    // DifferentialDriveOdometry is known to work on other robots.
    frc::DifferentialDriveOdometry m_odometer{frc::Rotation2d(0_rad)};

    // Design controller
    // States: [x position, y position, heading, left velocity, right velocity]
    Eigen::Matrix<double, 5, 2> m_B;
    Eigen::Matrix<double, 2, 5> m_K0;
    Eigen::Matrix<double, 2, 5> m_K1;

    // Controller reference
    Eigen::Matrix<double, 5, 1> m_r;

    Eigen::Matrix<double, 5, 1> m_nextR;
    Eigen::Matrix<double, 2, 1> m_cappedU;

    frc::Trajectory m_trajectory;
    frc::Pose2d m_goal;

    wpi::mutex m_trajectoryMutex;

    bool m_atReferences = false;
    bool m_isEnabled = false;

    // The loggers that generates the comma separated value files
    frc::CSVLogFile positionLogger{"Drivetrain Positions",
                                   "Estimated X (m)",
                                   "Estimated Y (m)",
                                   "X Ref (m)",
                                   "Y Ref (m)",
                                   "Measured Left Position (m)",
                                   "Measured Right Position (m)",
                                   "Estimated Left Position (m)",
                                   "Estimated Right Position (m)",
                                   "Odometry X (m)",
                                   "Odometry Y (m)"};
    frc::CSVLogFile angleLogger{"Drivetrain Angles", "Measured Heading (rad)",
                                "Estimated Heading (rad)", "Heading Ref (rad)",
                                "Angle Error (rad)"};
    frc::CSVLogFile velocityLogger{"Drivetrain Velocities",
                                   "Measured Left Velocity (m/s)",
                                   "Measured Right Velocity (m/s)",
                                   "Estimated Left Vel (m/s)",
                                   "Estimated Right Vel (m/s)",
                                   "Left Vel Ref (m/s)",
                                   "Right Vel Ref (m/s)"};
    frc::CSVLogFile voltageLogger{
        "Drivetrain Voltages",     "Left Voltage (V)",
        "Right Voltage (V)",       "Left Voltage Error (V)",
        "Right Voltage Error (V)", "Battery Voltage (V)"};
    frc::CSVLogFile errorCovLogger{
        "Drivetrain Error Covariances",
        "X Cov (m^2)",
        "Y Cov (m^2)",
        "Heading Cov (rad^2)",
        "Left Vel Cov ((m/s)^2)",
        "Right Vel Cov ((m/s)^2)",
        "Left Pos Cov (m^2)",
        "Right Pos Cov (m^2)",
        "Left Voltage Error Cov (V^2)",
        "Right Voltage Error Cov (V^2)",
        "Angle Error Cov (rad^2)",
    };

    /**
     * Constrains theta to within the range (-pi, pi].
     *
     * @param theta Angle to normalize
     */
    static constexpr double NormalizeAngle(double theta) {
        // Constrain theta to within (-3pi, pi)
        const int n_pi_pos = (theta + wpi::math::pi) / 2.0 / wpi::math::pi;
        theta -= n_pi_pos * 2.0 * wpi::math::pi;

        // Cut off the bottom half of the above range to constrain within
        // (-pi, pi]
        const int n_pi_neg = (theta - wpi::math::pi) / 2.0 / wpi::math::pi;
        theta -= n_pi_neg * 2.0 * wpi::math::pi;

        return theta;
    }

    /**
     * Converts velocity and curvature of drivetrain into left and right wheel
     * velocities.
     *
     * @param velocity Linear velocity of drivetrain chassis.
     * @param curvature Curvature of drivetrain arc.
     * @param trackWidth Track width of drivetrain.
     */
    static constexpr std::tuple<units::meters_per_second_t,
                                units::meters_per_second_t>
    ToWheelVelocities(units::meters_per_second_t velocity,
                      frc::curvature_t curvature, units::meter_t trackWidth) {
        // clang-format off
        // v = (v_r + v_l) / 2     (1)
        // w = (v_r - v_l) / (2r)  (2)
        // k = w / v               (3)
        //
        // v_l = v - wr
        // v_l = v - (vk)r
        // v_l = v(1 - kr)
        //
        // v_r = v + wr
        // v_r = v + (vk)r
        // v_r = v(1 + kr)
        // clang-format on
        auto vl = velocity * (1 - (curvature / 1_rad * trackWidth / 2.0));
        auto vr = velocity * (1 + (curvature / 1_rad * trackWidth / 2.0));
        return {vl, vr};
    }

    static void ScaleCapU(Eigen::Matrix<double, 2, 1>* u);
};
}  // namespace frc3512
