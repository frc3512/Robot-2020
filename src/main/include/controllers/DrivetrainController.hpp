// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <array>
#include <chrono>
#include <functional>
#include <tuple>
#include <vector>

#include <frc/controller/PlantInversionFeedforward.h>
#include <frc/estimator/ExtendedKalmanFilter.h>
#include <frc/logging/CSVLogFile.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <units/units.h>
#include <wpi/math>
#include <wpi/mutex.h>

#include "Constants.hpp"
#include "controllers/ControllerBase.hpp"

namespace frc3512 {

class DrivetrainController : public ControllerBase<10, 2, 3> {
public:
    static constexpr units::meter_t kWheelRadius = 3_in;
    static constexpr double kDriveGearRatio = 1.0 / 1.0;
    static constexpr double kDpP =
        (2.0 * wpi::math::pi * kWheelRadius.to<double>()) * kDriveGearRatio /
        2048.0;
    static constexpr units::meter_t kLength = 0.9398_m;
    static constexpr units::meter_t kWidth = 0.990405073902434_m;

    static constexpr std::array<double, 5> kControllerQ{0.0625, 0.125, 2.5,
                                                        0.95, 0.95};
    static constexpr std::array<double, 2> kControllerR{12.0, 12.0};

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
     * Constructs a drivetrain controller.
     */
    DrivetrainController();

    DrivetrainController(const DrivetrainController&) = delete;
    DrivetrainController& operator=(const DrivetrainController&) = delete;

    void Enable();
    void Disable();
    bool IsEnabled() const;

    void SetOpenLoop(bool manualControl);
    bool IsOpenLoop() const;

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

    /**
     * Set inputs.
     *
     * @param leftU Voltage applied to the left drivetrain
     * @param rightU Voltage applied to the right drivetrain
     */
    void SetMeasuredInputs(units::volt_t leftU, units::volt_t rightU);

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

    const Eigen::Matrix<double, 10, 1>& GetReferences() const override;

    const Eigen::Matrix<double, 10, 1>& GetStates() const override;

    const Eigen::Matrix<double, 2, 1>& GetInputs() const override;

    const Eigen::Matrix<double, 3, 1>& GetOutputs() const override;

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

    /**
     * Returns a trajectory config with a differential drive dynamics constraint
     * included.
     */
    frc::TrajectoryConfig MakeTrajectoryConfig() const;

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
    static constexpr double kPositionTolerance = 0.5;  // meters
    static constexpr double kVelocityTolerance = 2.0;  // meters/second
    static constexpr double kAngleTolerance = 0.52;    // radians
    static constexpr decltype(1_V / 1_mps) kLinearV = 3.02_V / 1_mps;
    static constexpr decltype(1_V / 1_mps_sq) kLinearA = 0.642_V / 1_mps_sq;
    static constexpr decltype(1_V / 1_rad_per_s) kAngularV =
        1.382_V / 1_rad_per_s;
    static constexpr decltype(1_V / (1_rad_per_s / 1_s)) kAngularA =
        (0.3398_V / 4.0) / (1_rad_per_s / 1_s);
    static constexpr units::meters_per_second_t kMaxV = 12_V / kLinearV;
    static constexpr decltype(1_mps_sq) kMaxA = 12_V / kLinearA;

    // Robot radius
    static constexpr auto rb = kWidth / 2.0;

    // The current voltage inputs
    Eigen::Matrix<double, 2, 1> m_appliedU;

    // The current sensor measurements
    Eigen::Matrix<double, 3, 1> m_localY;
    Eigen::Matrix<double, 6, 1> m_globalY;

    // Design observer. See the enums above for lists of the states, inputs, and
    // outputs.
    frc::ExtendedKalmanFilter<10, 2, 3> m_observer{
        Dynamics,
        LocalMeasurementModel,
        {0.002, 0.002, 0.0001, 1.5, 1.5, 0.5, 0.5, 10.0, 10.0, 2.0},
        {0.0001, 0.005, 0.005},
        Constants::kDt};
    frc::PlantInversionFeedforward<10, 2> m_ff{Dynamics, Constants::kDt};

    Eigen::Matrix<double, 5, 2> m_B;
    Eigen::Matrix<double, 2, 5> m_K0;
    Eigen::Matrix<double, 2, 5> m_K1;

    // Controller reference
    Eigen::Matrix<double, 10, 1> m_r;

    Eigen::Matrix<double, 10, 1> m_nextR;
    Eigen::Matrix<double, 2, 1> m_cappedU;

    frc::Trajectory m_trajectory;
    frc::Pose2d m_goal;

    wpi::mutex m_trajectoryMutex;

    bool m_atReferences = false;
    bool m_isEnabled = false;
    bool m_isOpenLoop = false;

    // The loggers that generates the comma separated value files
    frc::CSVLogFile positionLogger{"Drivetrain Positions",
                                   "Estimated X (m)",
                                   "Estimated Y (m)",
                                   "X Ref (m)",
                                   "Y Ref (m)",
                                   "Measured Left Position (m)",
                                   "Measured Right Position (m)",
                                   "Estimated Left Position (m)",
                                   "Estimated Right Position (m)"};
    frc::CSVLogFile angleLogger{"Drivetrain Angles", "Measured Heading (rad)",
                                "Estimated Heading (rad)", "Heading Ref (rad)",
                                "Angle Error (rad)"};
    frc::CSVLogFile velocityLogger{"Drivetrain Velocities",
                                   "Estimated Left Vel (m/s)",
                                   "Estimated Right Vel (m/s)",
                                   "Left Vel Ref (m/s)", "Right Vel Ref (m/s)"};
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
