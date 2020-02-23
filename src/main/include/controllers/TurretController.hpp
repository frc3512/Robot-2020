// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Eigen/Core>
#include <frc/DigitalInput.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/logging/CSVLogFile.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/system/plant/SingleJointedArmSystem.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/units.h>
#include <wpi/math>

#include "Constants.hpp"

namespace frc3512 {

class TurretController {
public:
    // State tolerances in radians and radians/sec respectively.
    static constexpr units::radian_t kAngleTolerance = 0.05_rad;
    static constexpr units::radians_per_second_t kAngularVelocityTolerance =
        2.0_rad_per_s;
    // Target model-points in the global frame, pulled from
    // https://files.slack.com/files-pri/T29CNG6MQ-FTV1L0XD1/img_4471.jpg
    static constexpr frc::Translation3d A{629.25_in, 108.464_in, 115.25_in};
    static constexpr frc::Translation3d B{629.25_in, 106.464_in, 115.25_in};
    static constexpr frc::Translation3d C{629.25_in, 98.644_in, 98.25_in};
    static constexpr frc::Translation3d D{629.25_in, 93.912_in, 100.25_in};
    static constexpr frc::Translation3d E{629.25_in, 80.75_in, 98.25_in};
    static constexpr frc::Translation3d F{629.25_in, 82.214_in, 100.25_in};
    static constexpr frc::Translation3d G{629.25_in, 69.214_in, 115.25_in};
    static constexpr frc::Translation3d H{629.25_in, 71.214_in, 115.25_in};

    TurretController();

    TurretController(const TurretController&) = delete;
    TurretController& operator=(const TurretController&) = delete;

    /**
     * Enables the control loop.
     */
    void Enable();

    /**
     * Disables the control loop.
     */
    void Disable();

    /**
     * Sets the end goal of the controller profile.
     *
     * @param goal Position in radians to set the goal to.
     */
    void SetGoal(units::radian_t angleGoal,
                 units::radians_per_second_t angularVelocityGoal);

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
     * Sets the current encoder measurement.
     *
     * @param measuredAngle Angle of the carriage in radians.
     */
    void SetMeasuredOutputs(units::radian_t angle);

    /**
     * Sets the current hard limit outputs based on whether if either the left
     * or right sensors were triggered.
     *
     * @param leftLimit Output of right hall effect, 'true' meaning triggered
     * @param rightLimit Output of left hall effect, 'true' meaning triggered
     */
    void SetHardLimitOutputs(bool leftLimit, bool rightLimit);

    /**
     * Sets the current estimated global pose of the drivetrain.
     */
    void SetDrivetrainStatus(const Eigen::Matrix<double, 10, 1>& nextXhat);

    /**
     * Returns the projected pose of the turret.
     */
    frc::Pose2d GetNextPose() const;

    /**
     * Returns the control loop calculated voltage.
     */
    units::volt_t ControllerVoltage() const;

    /**
     * Returns the estimated angle.
     */
    units::radian_t EstimatedAngle() const;

    /**
     * Returns the estimated angular velocity.
     */
    units::radians_per_second_t EstimatedAngularVelocity() const;

    /**
     * Returns the current angle reference.
     */
    units::radian_t AngleReference();

    /**
     * Returns the current angular velocity reference.
     */
    units::radians_per_second_t AngularVelocityReference();

    /**
     * Returns the error between the angle reference and the angle
     * estimate.
     */
    units::radian_t AngleError() const;

    /**
     * Returns the error between the angular velocity reference and the angular
     * velocity estimate.
     */
    units::radians_per_second_t AngularVelocityError() const;

    /**
     * Returns the angle the target must rotate to in the global frame to point
     * at the target.
     *
     * @param target Next timestep's X and Y of the target in the global frame
     * @param turret Next timestep's X and Y of the turret in the global frame
     */
    units::radian_t CalculateHeading(Eigen::Vector2d target,
                                     Eigen::Vector2d turret);

    /**
     * Executes the control loop for a cycle.
     */
    void Update(units::second_t dt, units::second_t elapsedTime);

    /**
     * Resets any internal state.
     */
    void Reset();

private:
    const frc::Translation3d targetModelCenter = (A + G) / 2.0;
    const frc::Pose2d targetPoseInGlobal{targetModelCenter.X(),
                                         targetModelCenter.Y(),
                                         units::radian_t{wpi::math::pi}};

    // The current sensor measurement.
    Eigen::Matrix<double, 1, 1> m_y;
    frc::TrapezoidProfile<units::radians>::State m_goal;

    frc::TrapezoidProfile<units::radians>::Constraints m_constraints{
        Constants::Turret::kMaxV, Constants::Turret::kMaxA};

    frc::TrapezoidProfile<units::radians>::State m_profiledReference;

    // frc::LinearSystem<2, 1, 1> m_plant =
    // frc::IdentifyPositionSystem(Constants::Turret::kV,
    // Constants::Turret::kA);
    frc::LinearSystem<2, 1, 1> m_plant = [=] {
        constexpr auto motor = frc::DCMotor::NEO();

        // Arm moment of inertia
        constexpr auto J = 0.1579_kg_sq_m;

        // Gear ratio
        constexpr double G = 160.0 / 18.0;

        return frc::SingleJointedArmSystem(motor, J, G);
    }();

    frc::LinearQuadraticRegulator<2, 1> m_lqr{
        m_plant, {0.01245, 0.109726}, {12.0}, Constants::kDt};
    frc::KalmanFilter<2, 1, 1> m_observer{
        m_plant, Constants::kDt, {0.21745, 0.28726}, {0.01}};

    Eigen::Matrix<double, 2, 1> m_nextR;

    bool m_atReferences = false;
    bool m_atLeftLimit = false;
    bool m_atRightLimit = false;

    frc::Pose2d m_drivetrainNextPoseInGlobal;
    Eigen::Matrix<double, 10, 1> m_drivetrainNextXhat;
    frc::Pose2d m_turretNextPoseInGlobal;

    frc::CSVLogFile positionLogger{
        "Turret Position", "Voltage (V)", "Measured Angle (rad)",
        "Estimated Angle (rad)", "Angle Reference (rad)"};
    frc::CSVLogFile velocityLogger{"Turret Velocity", "Voltage (V)",
                                   "Estimated Angular Velocity (rad/s)",
                                   "Angular Velocity Reference (rad/s)"};

    Eigen::Matrix<double, 1, 1> m_u;

    Eigen::Vector2d ToVector2d(frc::Translation2d translation);
};

}  // namespace frc3512
