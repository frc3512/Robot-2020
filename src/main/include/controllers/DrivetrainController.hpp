// Copyright (c) 2018-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <array>
#include <functional>
#include <tuple>
#include <vector>

#include <frc/controller/ControlAffinePlantInversionFeedforward.h>
#include <frc/geometry/Pose2d.h>
#include <frc/system/LinearSystem.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc2/Timer.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/curvature.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/math>

#include "Constants.hpp"
#include "LerpTable.hpp"
#include "controllers/ControllerBase.hpp"

namespace frc3512 {

class DrivetrainController : public ControllerBase<7, 2, 4> {
public:
    static constexpr units::meter_t kWheelRadius = 3.05_in;
    static constexpr double kDriveGearRatio = 1.0 / 1.0;
    static constexpr double kDpP =
        (2.0 * wpi::math::pi * kWheelRadius.to<double>()) * kDriveGearRatio /
        2048.0;

    static constexpr units::meter_t kWidth = [] {
        auto absoluteValue = [](auto arg) {
            return arg > decltype(arg){0} ? arg : -1.0 * arg;
        };

        // These values were collected by rotating the robot in place and
        // recording the encoder position and gyro heading measurements.
        // Difference is final measurement minus initial measurement.
        constexpr auto kLeftPosition = 2.18274_m - 0_m;
        constexpr auto kRightPosition = (-1.0 * 2.19665_m) - 0_m;
        constexpr auto kHeading = (-1.0 * 3.1219_rad) - 3.1415_rad;

        return (absoluteValue(kLeftPosition) + absoluteValue(kRightPosition)) /
               absoluteValue(kHeading) * 1_rad;
    }();

    static constexpr std::array<double, 5> kControllerQ{0.0625, 0.125, 2.5,
                                                        0.95, 0.95};
    static constexpr std::array<double, 2> kControllerR{12.0, 12.0};

    static constexpr auto kLinearV = 3.02_V / 1_mps;
    static constexpr auto kLinearA = 0.642_V / 1_mps_sq;
    static constexpr auto kAngularV = 1.382_V / 1_rad_per_s;
    static constexpr auto kAngularA = 0.08495_V / 1_rad_per_s_sq;
    static constexpr auto kMaxV = 12_V / kLinearV;
    static constexpr auto kMaxA = 12_V / kLinearA;

    /**
     * States of the drivetrain system.
     */
    class State {
    public:
        static constexpr int kX = 0;
        static constexpr int kY = 1;
        static constexpr int kHeading = 2;
        static constexpr int kLeftVelocity = 3;
        static constexpr int kRightVelocity = 4;
        static constexpr int kLeftPosition = 5;
        static constexpr int kRightPosition = 6;
    };

    /**
     * Inputs of the drivetrain system.
     */
    class Input {
    public:
        static constexpr int kLeftVoltage = 0;
        static constexpr int kRightVoltage = 1;
    };

    /**
     * Local outputs of the drivetrain system.
     */
    class LocalOutput {
    public:
        static constexpr int kHeading = 0;
        static constexpr int kLeftPosition = 1;
        static constexpr int kRightPosition = 2;
        static constexpr int kAccelerationX = 3;
        static constexpr int kAccelerationY = 4;
    };

    /**
     * Global outputs of the drivetrain system.
     */
    class GlobalOutput {
    public:
        static constexpr int kX = 0;
        static constexpr int kY = 1;
    };

    /**
     * Constructs a drivetrain controller.
     */
    DrivetrainController();

    DrivetrainController(DrivetrainController&&) = default;
    DrivetrainController& operator=(DrivetrainController&&) = default;

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
    void AddTrajectory(
        const frc::Pose2d& start,
        const std::vector<frc::Translation2d>& interior, const frc::Pose2d& end,
        const frc::TrajectoryConfig& config = MakeTrajectoryConfig());

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
    void AddTrajectory(
        const std::vector<frc::Pose2d>& waypoints,
        const frc::TrajectoryConfig& config = MakeTrajectoryConfig());

    /**
     * Returns true if drivetrain controller has a trajectory to follow.
     */
    bool HaveTrajectory() const;

    /**
     * Abort trajectory tracking.
     */
    void AbortTrajectories();

    /**
     * Returns whether the drivetrain controller is at the goal waypoint.
     */
    bool AtGoal() const;

    /**
     * Resets any internal state.
     *
     * @param initialPose Initial pose for state estimate.
     */
    void Reset(const frc::Pose2d& initialPose);

    Eigen::Matrix<double, 2, 1> Calculate(
        const Eigen::Matrix<double, 7, 1>& x) override;

    /**
     * Returns the drivetrain's plant.
     */
    static frc::LinearSystem<2, 2, 2> GetPlant();

    /**
     * Returns a TrajectoryConfig containing a differential drive dynamics
     * constraint.
     */
    static frc::TrajectoryConfig MakeTrajectoryConfig();

    /**
     * Returns the linear time-varying controller gain for the given state.
     *
     * @param x The state vector.
     */
    Eigen::Matrix<double, 2, 5> ControllerGainForState(
        const Eigen::Matrix<double, 7, 1>& x);

    Eigen::Matrix<double, 2, 1> Controller(
        const Eigen::Matrix<double, 7, 1>& x,
        const Eigen::Matrix<double, 7, 1>& r);

    static Eigen::Matrix<double, 7, 1> Dynamics(
        const Eigen::Matrix<double, 7, 1>& x,
        const Eigen::Matrix<double, 2, 1>& u);

    /**
     * Returns the Jacobian of the pose and velocity dynamics with respect to
     * the state vector.
     */
    static Eigen::Matrix<double, 5, 5> JacobianX(
        const Eigen::Matrix<double, 7, 1>& x);

    /**
     * Returns the Jacobian of the pose and velocity dynamics with respect to
     * the input vector.
     */
    static Eigen::Matrix<double, 5, 2> JacobianU(
        const Eigen::Matrix<double, 2, 1>& u);

    static Eigen::Matrix<double, 5, 1> LocalMeasurementModel(
        const Eigen::Matrix<double, 7, 1>& x,
        const Eigen::Matrix<double, 2, 1>& u);

    static Eigen::Matrix<double, 2, 1> GlobalMeasurementModel(
        const Eigen::Matrix<double, 7, 1>& x,
        const Eigen::Matrix<double, 2, 1>& u);

private:
    static constexpr double kPositionTolerance = 0.5;  // meters
    static constexpr double kVelocityTolerance = 2.0;  // meters/second
    static constexpr double kAngleTolerance = 0.52;    // radians

    static const frc::LinearSystem<2, 2, 2> kPlant;

    frc::ControlAffinePlantInversionFeedforward<7, 2> m_ff{Dynamics,
                                                           Constants::kDt};

    Eigen::Matrix<double, 5, 5> m_A;
    Eigen::Matrix<double, 5, 2> m_B;
    Eigen::Matrix<double, 5, 5> m_Q;
    Eigen::Matrix<double, 2, 2> m_R;

    // LUT from drivetrain linear velocity to LQR gain
    LerpTable<units::meters_per_second_t, Eigen::Matrix<double, 2, 5>> m_table;

    frc::Trajectory m_trajectory;
    frc::Pose2d m_goal;
    frc2::Timer m_trajectoryTimeElapsed;

    bool m_atReferences = false;

    /**
     * Update "at references" flag based on next reference and current state
     * estimate.
     *
     * @param error The error vector.
     */
    void UpdateAtReferences(const Eigen::Matrix<double, 5, 1>& error);

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
                      units::curvature_t curvature, units::meter_t trackWidth) {
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
};
}  // namespace frc3512
