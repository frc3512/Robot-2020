// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/controller/LinearPlantInversionFeedforward.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/logging/CSVLogFile.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/LinearSystemLoop.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/units.h>
#include <wpi/math>

#include "Constants.hpp"
#include "TargetModel.hpp"
#include "controllers/ControllerBase.hpp"

namespace frc3512 {

class TurretController : public ControllerBase<2, 1, 1> {
public:
    static constexpr double kGearRatio = 18.0 / 160.0;
    static constexpr double kDpR = kGearRatio * 2.0 * wpi::math::pi;

    // Transformation from drivetrain to turret
    static const frc::Pose2d kDrivetrainToTurretFrame;

    // State tolerances
    static constexpr units::radian_t kAngleTolerance = 0.05_rad;
    static constexpr units::radians_per_second_t kAngularVelocityTolerance =
        2.0_rad_per_s;

    class State {
    public:
        static constexpr int kAngle = 0;
        static constexpr int kAngularVelocity = 1;
    };

    class Input {
    public:
        static constexpr int kVoltage = 0;
    };

    class Output {
    public:
        static constexpr int kAngle = 0;
    };

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

    const Eigen::Matrix<double, 2, 1>& GetReferences() const override;

    const Eigen::Matrix<double, 2, 1>& GetStates() const override;

    const Eigen::Matrix<double, 1, 1>& GetInputs() const override;

    const Eigen::Matrix<double, 1, 1>& GetOutputs() const override;

    void UpdateController(units::second_t dt) override;

    /**
     * Returns the turret plant.
     */
    const frc::LinearSystem<2, 1, 1>& GetPlant() const;

    /**
     * Returns the projected pose of the turret.
     */
    frc::Pose2d GetNextPose() const;

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
     * Resets any internal state.
     */
    void Reset();

private:
    static constexpr decltype(1_V / 1_rad_per_s) kV = 4.42_V / 1_rad_per_s;
    static constexpr decltype(1_V / (1_rad_per_s / 1_s)) kA =
        0.14_V / (1_rad_per_s / 1_s);
    static constexpr units::radians_per_second_t kMaxV = 1.477996_rad_per_s;
    static constexpr decltype(1_rad_per_s / 1_s) kMaxA =
        7.782482_rad_per_s / 1_s;

    const frc::Pose2d targetPoseInGlobal{TargetModel::kCenter.X(),
                                         TargetModel::kCenter.Y(),
                                         units::radian_t{wpi::math::pi}};

    // The current sensor measurement.
    Eigen::Matrix<double, 1, 1> m_y;
    frc::TrapezoidProfile<units::radians>::State m_goal;

    frc::TrapezoidProfile<units::radians>::Constraints m_constraints{kMaxV,
                                                                     kMaxA};

    frc::TrapezoidProfile<units::radians>::State m_profiledReference;

    frc::LinearSystem<2, 1, 1> m_plant =
        frc::LinearSystemId::IdentifyPositionSystem(kV.to<double>(),
                                                    kA.to<double>());

    frc::LinearQuadraticRegulator<2, 1> m_lqr{
        m_plant, {0.01245, 0.109726}, {12.0}, Constants::kDt};
    frc::LinearPlantInversionFeedforward<2, 1> m_ff{m_plant, Constants::kDt};
    frc::KalmanFilter<2, 1, 1> m_observer{
        m_plant, {0.21745, 0.28726}, {0.01}, Constants::kDt};
    frc::LinearSystemLoop<2, 1, 1> m_loop{m_plant, m_lqr, m_ff, m_observer,
                                          12_V};

    Eigen::Matrix<double, 2, 1> m_nextR;

    bool m_atReferences = false;
    bool m_atLeftLimit = false;
    bool m_atRightLimit = false;
    bool m_isEnabled = false;

    frc::Pose2d m_drivetrainNextPoseInGlobal;
    Eigen::Matrix<double, 10, 1> m_drivetrainNextXhat;
    frc::Pose2d m_turretNextPoseInGlobal;

    Eigen::Matrix<double, 1, 1> m_u;

    static Eigen::Vector2d ToVector2d(frc::Translation2d translation);
};

}  // namespace frc3512
