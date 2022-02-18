// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <functional>
#include <tuple>
#include <vector>

#include <frc/Timer.h>
#include <frc/controller/ControlAffinePlantInversionFeedforward.h>
#include <frc/controller/LTVDiffDriveController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/system/LinearSystem.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <units/curvature.h>
#include <units/length.h>
#include <units/velocity.h>
#include <wpi/numbers>

#include "Constants.hpp"
#include "controllers/ControllerBase.hpp"

namespace frc3512 {

/**
 * The drivetrain controller.
 *
 * The drivetrain uses a linear time-varying LQR for feedback control. Since the
 * model is control-affine (the dynamics are nonlinear, but the control inputs
 * provide a linear contribution), a plant inversion feedforward was used.
 * Trajectories generated from splines provide the motion profile to follow.
 *
 * The linear time-varying controller has a similar form to the LQR, but the
 * model used to compute the controller gain is the nonlinear model linearized
 * around the drivetrain's current state. We precomputed gains for important
 * places in our state-space, then interpolated between them with a LUT to save
 * computational resources.
 *
 * We decided to control for longitudinal error and cross-track error in the
 * chassis frame instead of x and y error in the global frame, so the state
 * Jacobian simplified such that we only had to sweep velocities (-4m/s to
 * 4m/s).
 *
 * See section 9.6 in Controls Engineering in FRC for a derivation of the
 * control law we used shown in theorem 9.6.3.
 */
class DrivetrainController : public ControllerBase<7, 2, 4> {
public:
    /// The wheel radius.
    static constexpr units::meter_t kWheelRadius = 3.05_in;

    /// The drivetrain gear ratio from the encoder to the wheel.
    static constexpr double kDriveGearRatio = 1.0 / 1.0;

    /// Drivetrain distance per encoder pulse.
    static constexpr double kDpP =
        (2.0 * wpi::numbers::pi * kWheelRadius.value()) * kDriveGearRatio /
        2048.0;

    /// Drivetrain chassis width.
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

    /// Linear velocity system ID gain.
    static constexpr auto kLinearV = 3.02_V / 1_mps;

    /// Linear acceleration system ID gain.
    static constexpr auto kLinearA = 0.642_V / 1_mps_sq;

    /// Angular velocity system ID gain.
    static constexpr auto kAngularV = 1.382_V / 1_mps;

    /// Angular acceleration system ID gain.
    static constexpr auto kAngularA = 0.08495_V / 1_mps_sq;

    /// Maximum linear velocity.
    static constexpr auto kMaxV = 12_V / kLinearV;

    /// Maximum linear acceleration.
    static constexpr auto kMaxA = 12_V / kLinearA;

    /**
     * States of the drivetrain system.
     */
    class State {
    public:
        /// X position in global coordinate frame.
        static constexpr int kX = 0;

        /// Y position in global coordinate frame.
        static constexpr int kY = 1;

        /// Heading in global coordinate frame.
        static constexpr int kHeading = 2;

        /// Left encoder velocity.
        static constexpr int kLeftVelocity = 3;

        /// Right encoder velocity.
        static constexpr int kRightVelocity = 4;

        /// Left encoder position.
        static constexpr int kLeftPosition = 5;

        /// Right encoder position.
        static constexpr int kRightPosition = 6;
    };

    /**
     * Inputs of the drivetrain system.
     */
    class Input {
    public:
        /// Left motor voltage.
        static constexpr int kLeftVoltage = 0;

        /// Right motor voltage.
        static constexpr int kRightVoltage = 1;
    };

    /**
     * Local outputs of the drivetrain system.
     */
    class LocalOutput {
    public:
        /// Heading in global coordinate frame.
        static constexpr int kHeading = 0;

        /// Left encoder position.
        static constexpr int kLeftPosition = 1;

        /// Right encoder position.
        static constexpr int kRightPosition = 2;

        /// Acceleration along X axis in chassis coordinate frame.
        static constexpr int kAccelerationX = 3;

        /// Acceleration along Y axis in chassis coordinate frame.
        static constexpr int kAccelerationY = 4;
    };

    /**
     * Global outputs of the drivetrain system.
     */
    class GlobalOutput {
    public:
        /// X position
        static constexpr int kX = 0;

        /// Y position
        static constexpr int kY = 1;
    };

    /**
     * Constructs a drivetrain controller.
     */
    DrivetrainController();

    /**
     * Move constructor.
     */
    DrivetrainController(DrivetrainController&&) = default;

    /**
     * Move assignment operator.
     */
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

    /**
     * Returns the next output of the controller.
     *
     * @param x The current state x.
     */
    Eigen::Vector<double, 2> Calculate(
        const Eigen::Vector<double, 7>& x) override;

    /**
     * Returns the drivetrain's plant.
     */
    static frc::LinearSystem<2, 2, 2> GetPlant();

    /**
     * Returns a TrajectoryConfig containing a differential drive dynamics
     * constraint with the start and end velocities set to zero.
     */
    static frc::TrajectoryConfig MakeTrajectoryConfig();

    /**
     * Returns a TrajectoryConfig containing a differential drive dynamics
     * constraint and the specified start and end velocities.
     *
     * @param startVelocity The start velocity of the trajectory config.
     * @param endVelocity The end velocity of the trajectory config.
     */
    static frc::TrajectoryConfig MakeTrajectoryConfig(
        units::meters_per_second_t startVelocity,
        units::meters_per_second_t endVelocity);

    /**
     * The drivetrain system dynamics.
     *
     * @param x The state vector.
     * @param u The input vector.
     */
    static Eigen::Vector<double, 7> Dynamics(const Eigen::Vector<double, 7>& x,
                                             const Eigen::Vector<double, 2>& u);

    /**
     * Returns the local measurements that correspond to the given state and
     * input vectors.
     *
     * @param x The state vector.
     * @param u The input vector.
     */
    static Eigen::Vector<double, 5> LocalMeasurementModel(
        const Eigen::Vector<double, 7>& x, const Eigen::Vector<double, 2>& u);

    /**
     * Returns the global measurements that correspond to the given state and
     * input vectors.
     *
     * @param x The state vector.
     * @param u The input vector.
     */
    static Eigen::Vector<double, 2> GlobalMeasurementModel(
        const Eigen::Vector<double, 7>& x, const Eigen::Vector<double, 2>& u);

private:
    static constexpr auto kPositionTolerance = 0.5_m;
    static constexpr auto kVelocityTolerance = 2_mps;
    static constexpr auto kAngleTolerance = 0.52_rad;

    static const frc::LinearSystem<2, 2, 2> kPlant;

    frc::ControlAffinePlantInversionFeedforward<7, 2> m_ff{
        Dynamics, Constants::kControllerPeriod};
    frc::LTVDiffDriveController m_fb{kPlant,
                                     kWidth,
                                     {0.0625, 0.125, 2.5, 0.95, 0.95},
                                     {12.0, 12.0},
                                     Constants::kControllerPeriod};

    frc::Trajectory m_trajectory;
    frc::Pose2d m_goal;
    frc::Timer m_trajectoryTimeElapsed;

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
        auto vl = velocity * (1 - (curvature / 1_rad * trackWidth / 2.0));
        auto vr = velocity * (1 + (curvature / 1_rad * trackWidth / 2.0));
        return {vl, vr};
    }
};
}  // namespace frc3512
