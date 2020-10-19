// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/controller/LinearPlantInversionFeedforward.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation3d.h>
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
    static constexpr auto kAngleTolerance = 0.05_rad;
    static constexpr auto kAngularVelocityTolerance = 2.0_rad_per_s;

    // Turret configuration space limits
    static constexpr units::radian_t kCCWLimit{wpi::math::pi / 2.0};
    static constexpr units::radian_t kCWLimit{-wpi::math::pi / 2.0};

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
     * Sets the current estimated global pose of the drivetrain.
     */
    void SetDrivetrainStates(const Eigen::Matrix<double, 10, 1>& x);

    const Eigen::Matrix<double, 2, 1>& GetReferences() const override;

    const Eigen::Matrix<double, 2, 1>& GetStates() const override;

    Eigen::Matrix<double, 1, 1> Update(const Eigen::Matrix<double, 1, 1>& y,
                                       units::second_t dt) override;

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
     * Returns the angular velocity the turret should follow to stay pointing at
     * the target.
     *
     * @param v Next timestep's velocity vector of turret in the global frame.
     * @param r Next timestep's translation from the turret to the target in the
     *          global frame.
     */
    units::radians_per_second_t CalculateAngularVelocity(Eigen::Vector2d v,
                                                         Eigen::Vector2d r);

    /**
     * Resets any internal state.
     */
    void Reset();

private:
    static constexpr auto kV = 4.42_V / 1_rad_per_s;
    static constexpr auto kA = 0.14_V / 1_rad_per_s_sq;
    static constexpr auto kMaxV = 1.477996_rad_per_s;
    static constexpr auto kMaxA = 7.782482_rad_per_s_sq;

    const frc::Pose2d targetPoseInGlobal{TargetModel::kCenter.X(),
                                         TargetModel::kCenter.Y(),
                                         units::radian_t{wpi::math::pi}};

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

    // Controller reference
    Eigen::Matrix<double, 2, 1> m_r;
    Eigen::Matrix<double, 2, 1> m_nextR;

    bool m_atReferences = false;

    frc::Pose2d m_drivetrainNextPoseInGlobal;
    units::meters_per_second_t m_drivetrainLeftVelocity = 0_mps;
    units::meters_per_second_t m_drivetrainRightVelocity = 0_mps;
    frc::Pose2d m_turretNextPoseInGlobal;

    static Eigen::Vector2d ToVector2d(frc::Translation2d translation);

    template <typename Unit>
    static Eigen::Vector2d ToVector2d(Unit x, Unit y) {
        Eigen::Vector2d result;
        result << x.template to<double>(), y.template to<double>();
        return result;
    }
};

}  // namespace frc3512
