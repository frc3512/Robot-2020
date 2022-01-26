// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <functional>
#include <tuple>
#include <vector>

#include <frc/Timer.h>
#include <frc/controller/ControlAffinePlantInversionFeedforward.h>
#include <frc/controller/LinearPlantInversionFeedforward.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/system/LinearSystem.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/curvature.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/numbers>

#include "Constants.hpp"
#include "LerpTable.hpp"
#include "controllers/ControllerBase.hpp"

namespace frc3512 {
/**
 * The drivetrain turning controller.
 */
class DrivetrainTurningController : public ControllerBase<7, 2, 2> {
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

    /// Linear velocity system ID gain.
    static constexpr auto kTLinearV = 3.02_V / 1_rad_per_s;

    /// Linear acceleration system ID gain.
    static constexpr auto kTLinearA = 0.642_V / 1_rad_per_s_sq;

    /// Angular velocity system ID gain.
    static constexpr auto kTAngularV = 1.382_V / 1_rad_per_s;

    /// Angular acceleration system ID gain.
    static constexpr auto kTAngularA = 0.08495_V / 1_rad_per_s_sq;

    /// Maximum linear velocity.
    static constexpr auto kTMaxV = 12_V / kTLinearV;

    /// Maximum linear acceleration.
    static constexpr auto kTMaxA = 12_V / kTLinearA;

    /**
     * States of the drivetrain system.
     */
    class State {
    public:
        /// Heading in global coordinate frame.
        static constexpr int kHeading = 0;

        /// Left encoder velocity.
        static constexpr int kLeftVelocity = 1;

        /// Right encoder velocity.
        static constexpr int kRightVelocity = 2;
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
     * Constructs a drivetrain turning controller
     */
    DrivetrainTurningController();

    /**
     * Move constructor.
     */
    DrivetrainTurningController(DrivetrainTurningController&&) = default;

    /**
     * Move assignment operator.
     */
    DrivetrainTurningController& operator=(DrivetrainTurningController&&) =
        default;

    /**
     * Sets the heading goal for the drivetrain to achieve
     *
     * @param newHeading   The new heading to set as the goal
     */
    void SetHeadingGoal(units::radian_t newHeading);

    /**
     * Returns true if drivetrain controller has a new heading goal.
     */
    bool HaveHeadingGoal() const;

    /**
     *  Abort turn in place actions.
     */
    void AbortTurnInPlace();

    /**
     * Returns whether the drivetrain controller is at the goal heading.
     */
    bool AtHeading() const;

    /**
     * Returns the next output of the turn-in-place controller.
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
     * The drivetrain system dynamics for turning in place.
     */
    static frc::LinearSystem<3, 2, 3> TurningDynamics();

private:
    static constexpr double kPositionTolerance = 0.5;  // meters
    static constexpr double kVelocityTolerance = 2.0;  // meters/second
    static constexpr double kAngleTolerance = 0.52;    // radians

    static const frc::LinearSystem<2, 2, 2> kPlant;

    frc::LinearQuadraticRegulator<3, 2> m_turningLQR{
        TurningDynamics(),
        {0.001, 4.0, 4.0},
        {12.0, 12.0},
        Constants::kControllerPeriod};
    frc::LinearPlantInversionFeedforward<3, 2> m_turningFF{
        TurningDynamics(), Constants::kControllerPeriod};

    frc::TrapezoidProfile<units::radian>::State m_headingState;
    frc::TrapezoidProfile<units::radian>::Constraints m_constraints{kTMaxV,
                                                                    kTMaxA};
    frc::TrapezoidProfile<units::radian>::State m_profiledReference;

    units::radian_t m_headingGoal;

    bool m_atReferences = false;

    /**
     * Update "at references" flag based on next reference and current state
     * estimate.
     *
     * @param error The error vector.
     */
    void UpdateAtReferences(const Eigen::Vector<double, 5>& error);
};
}  // namespace frc3512
