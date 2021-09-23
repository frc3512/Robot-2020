// Copyright (c) 2018-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/controller/LinearPlantInversionFeedforward.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Velocity2d.h>
#include <frc/system/LinearSystem.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/numbers>

#include "Constants.hpp"
#include "controllers/ControllerBase.hpp"

namespace frc3512 {

/**
 * The turret controller.
 *
 * The turret uses an LQR for feedback control. When the drivetrain pose isn't
 * being used for aiming (which provides an implicit motion profile), a
 * trapezoidal motion profile is applied to references so a plant inversion
 * feedforward can be effectively applied at all times.
 *
 * The heading and angular rate references take into account the drivetrain's
 * current velocity so it won't overshoot as the drivetrain moves.
 */
class TurretController : public ControllerBase<2, 1, 1> {
public:
    /**
     * Turret control mode.
     */
    enum class ControlMode {
        /// Open loop
        kManual,
        /// Closed loop without auto-aiming
        kClosedLoop,
        /// Closed loop with auto-aiming
        kAutoAim,
        /// Closed loop with Vision data
        kVisionAim
    };

    /**
     * Gear ratio from encoder shaft to turret sprocket.
     *
     * There's a 1:8 ratio from the encoder shaft to the motor shaft, 18 teeth
     * on the motor sprocket, and 160 teeth on the turret sprocket.
     *
     * See docs/turret-encoder-resolution.md for encoder gear ratio selection.
     */
    static constexpr double kGearRatio = 8.0 * 18.0 / 160.0;

    /// Turret angle per encoder revolution
    static constexpr double kDpR = kGearRatio * 2.0 * wpi::numbers::pi;

    /// Transformation from drivetrain to turret
    static const frc::Pose2d kDrivetrainToTurretFrame;

    /// Couunterclockwise configuration space limit
    static constexpr units::radian_t kCCWLimit{2.0 / 3.0 * wpi::numbers::pi};

    /// Clockwise configuration space limit
    static constexpr units::radian_t kCWLimit{-2.0 / 3.0 * wpi::numbers::pi};

    /**
     * States of the turret system.
     */
    class State {
    public:
        /// Turret heading in turret coordinate frame.
        static constexpr int kAngle = 0;

        /// Angular velocity.
        static constexpr int kAngularVelocity = 1;
    };

    /**
     * Inputs of the turret system.
     */
    class Input {
    public:
        /// Motor voltage.
        static constexpr int kVoltage = 0;
    };

    /**
     * Outputs of the turret system.
     */
    class Output {
    public:
        /// Turret heading in turret coordinate frame.
        static constexpr int kAngle = 0;
    };

    TurretController();

    /**
     * Move constructor.
     */
    TurretController(TurretController&&) = default;

    /**
     * Move assignment operator.
     */
    TurretController& operator=(TurretController&&) = default;

    /**
     * Sets the end goal of the controller profile.
     *
     * @param angle           Goal angle.
     * @param angularVelocity Goal angular velocity.
     */
    void SetGoal(units::radian_t angle,
                 units::radians_per_second_t angularVelocity);

    /**
     * Sets the references.
     *
     * @param angle  Angle of the carriage in radians.
     * @param angularVelocity  Angular velocity of the carriage in radians per
     *                         second.
     */
    void SetReferences(units::radian_t angle,
                       units::radians_per_second_t angularVelocity);

    /**
     * Returns whether or not position and velocity are tracking the profile.
     */
    bool AtReferences() const;

    /**
     * Returns whether or not the goal has been reached.
     */
    bool AtGoal() const;

    /**
     * Sets the current estimated global pose of the drivetrain.
     */
    void SetDrivetrainStates(const Eigen::Matrix<double, 7, 1>& x);

    /**
     * Sets the current flywheel angular velocity reference.
     */
    void SetFlywheelReferences(units::radians_per_second_t r);

    /**
     * Sets turret control mode.
     *
     * @param mode Control mode.
     */
    void SetControlMode(ControlMode mode);

    /**
     * Set yaw and timestamp of the vision measurment.
     *
     * @param yaw yaw measurement in radians from the vision subsystem.
     * @param timestamp the corresponding timestamp for the yaw measurement.
     */
    void SetVisionMeasurements(units::radian_t yaw, units::second_t timestamp);

    /**
     * Returns currently set control mode.
     */
    ControlMode GetControlMode() const;

    /**
     * Resets any internal state.
     *
     * @param initialHeading The initial turret heading in the drivetrain frame.
     */
    void Reset(units::radian_t initialHeading);

    /**
     * Returns the next output of the controller.
     *
     * @param x The current state x.
     */
    Eigen::Matrix<double, 1, 1> Calculate(
        const Eigen::Matrix<double, 2, 1>& x) override;

    /**
     * Returns the turret plant.
     */
    static frc::LinearSystem<2, 1, 1> GetPlant();

    /**
     * Returns the angle the target must rotate to in the global frame to point
     * at the target.
     *
     * @param targetInGlobal Next timestep's X and Y of the target in the
     *                       global frame.
     * @param turretInGlobal Next timestep's X and Y of the turret in the global
     *                       frame.
     */
    units::radian_t CalculateHeading(frc::Translation2d targetInGlobal,
                                     frc::Translation2d turretInGlobal);

    /**
     * Returns the heading adjustment needed to hit a moving target.
     *
     * @param turretTranslationInGlobal Turret translation to the target in the
     *                                  global frame.
     * @param drivetrainVelocity        Drivetrain velocity vector.
     * @param flywheelAngularSpeed      Flywheel angular speed.
     */
    static units::radian_t CalculateHeadingAdjustment(
        frc::Translation2d turretTranslationInGlobal,
        frc::Velocity2d drivetrainVelocity,
        units::radians_per_second_t flywheelAngularSpeed);

    /**
     * Returns the angular velocity the turret should follow to stay pointing at
     * the target.
     *
     * @param v Next timestep's velocity vector of turret in the global frame.
     * @param r Next timestep's translation from the turret to the target in the
     *          global frame.
     */
    units::radians_per_second_t CalculateAngularVelocity(frc::Velocity2d v,
                                                         frc::Translation2d r);

    /**
     * Transforms drivetrain pose in global frame to turret pose in global
     * frame.
     *
     * @param drivetrainInGlobal Drivetrain pose in global frame.
     */
    static frc::Pose2d DrivetrainToTurretInGlobal(
        const frc::Pose2d& drivetrainInGlobal);

private:
    static constexpr auto kV = 4.42_V / 1_rad_per_s;
    static constexpr auto kA = 0.14_V / 1_rad_per_s_sq;
    static constexpr auto kMaxV = 12_V / kV;
    static constexpr auto kMaxA = 10_V / kA;

    // State tolerances
    static constexpr auto kAngleTolerance = 0.05_rad;
    static constexpr auto kAngularVelocityTolerance = 2.0_rad_per_s;

    frc::TrapezoidProfile<units::radian>::State m_goal;
    frc::TrapezoidProfile<units::radian>::Constraints m_constraints{kMaxV,
                                                                    kMaxA};
    frc::TrapezoidProfile<units::radian>::State m_profiledReference;

    frc::LinearSystem<2, 1, 1> m_plant{GetPlant()};
    frc::LinearQuadraticRegulator<2, 1> m_lqr{
        m_plant, {0.01245, 0.109726}, {12.0}, Constants::kControllerPeriod};
    frc::LinearPlantInversionFeedforward<2, 1> m_ff{
        m_plant, Constants::kControllerPeriod};

    units::radian_t m_visionYaw;
    units::second_t m_timestamp;

    ControlMode m_controlMode = ControlMode::kManual;

    bool m_atReferences = false;

    frc::Pose2d m_drivetrainNextPoseInGlobal;
    units::meters_per_second_t m_drivetrainLeftVelocity = 0_mps;
    units::meters_per_second_t m_drivetrainRightVelocity = 0_mps;

    units::radians_per_second_t m_flywheelAngularVelocityRef = 0_rad_per_s;

    /**
     * Update internal m_atReferences variable using next reference and current
     * state.
     *
     * @param error The error vector.
     */
    void UpdateAtReferences(const Eigen::Matrix<double, 2, 1>& error);
};

}  // namespace frc3512
