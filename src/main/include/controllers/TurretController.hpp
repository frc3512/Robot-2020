// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/controller/LinearPlantInversionFeedforward.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Velocity2d.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/math>

#include "Constants.hpp"
#include "controllers/ControllerBase.hpp"

namespace frc3512 {

class TurretController : public ControllerBase<2, 1, 1> {
public:
    static constexpr double kGearRatio = 18.0 / 160.0;
    static constexpr double kDpR = kGearRatio * 2.0 * wpi::math::pi;

    // Transformation from drivetrain to turret
    static const frc::Pose2d kDrivetrainToTurretFrame;

    // State tolerances
    static constexpr auto kAngleTolerance = 0.05_rad;
    static constexpr auto kAngularVelocityTolerance = 2.0_rad_per_s;

    // Turret configuration space limits
    static constexpr units::radian_t kCCWLimit{wpi::math::pi / 2.0};
    static constexpr units::radian_t kCWLimit{-wpi::math::pi / 2.0};

    /**
     * States of the turret system.
     */
    class State {
    public:
        static constexpr int kAngle = 0;
        static constexpr int kAngularVelocity = 1;
    };

    /**
     * Inputs of the turret system.
     */
    class Input {
    public:
        static constexpr int kVoltage = 0;
    };

    /**
     * Outputs of the turret system.
     */
    class Output {
    public:
        static constexpr int kAngle = 0;
    };

    TurretController();

    TurretController(const TurretController&) = delete;
    TurretController& operator=(const TurretController&) = delete;

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

    const Eigen::Matrix<double, 2, 1>& GetReferences() const override;

    const Eigen::Matrix<double, 2, 1>& GetStates() const override;

    /**
     * Resets any internal state.
     *
     * @param initialHeading The initial turret heading in the drivetrain frame.
     */
    void Reset(units::radian_t initialHeading);

    Eigen::Matrix<double, 1, 1> Update(const Eigen::Matrix<double, 1, 1>& y,
                                       units::second_t dt) override;

    /**
     * Returns the turret plant.
     */
    const frc::LinearSystem<2, 1, 1>& GetPlant() const;

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
    static constexpr auto kMaxV = 1.477996_rad_per_s;
    static constexpr auto kMaxA = 7.782482_rad_per_s_sq;

    static const frc::Pose2d kTargetPoseInGlobal;

    frc::TrapezoidProfile<units::radian>::State m_goal;
    frc::TrapezoidProfile<units::radian>::Constraints m_constraints{kMaxV,
                                                                    kMaxA};
    frc::TrapezoidProfile<units::radian>::State m_profiledReference;

    frc::LinearSystem<2, 1, 1> m_plant =
        frc::LinearSystemId::IdentifyPositionSystem<units::radian>(kV, kA);

    frc::LinearQuadraticRegulator<2, 1> m_lqr{
        m_plant, {0.01245, 0.109726}, {12.0}, Constants::kDt};
    frc::LinearPlantInversionFeedforward<2, 1> m_ff{m_plant, Constants::kDt};
    frc::KalmanFilter<2, 1, 1> m_observer{
        m_plant, {0.21745, 0.28726}, {0.01}, Constants::kDt};
    frc::LinearSystemLoop<2, 1, 1> m_loop{m_plant, m_lqr, m_ff, m_observer,
                                          12_V};

    // Controller reference
    Eigen::Matrix<double, 2, 1> m_r;
    Eigen::Matrix<double, 2, 1> m_nextR;

    bool m_atReferences = false;

    frc::Pose2d m_drivetrainNextPoseInGlobal;
    units::meters_per_second_t m_drivetrainLeftVelocity = 0_mps;
    units::meters_per_second_t m_drivetrainRightVelocity = 0_mps;

    units::radians_per_second_t m_flywheelAngularVelocityRef = 0_rad_per_s;
};

}  // namespace frc3512
