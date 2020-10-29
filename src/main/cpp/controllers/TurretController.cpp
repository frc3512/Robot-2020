// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#include "controllers/TurretController.hpp"

#include <frc/RobotController.h>

#include "controllers/DrivetrainController.hpp"
#include "controllers/NormalizeAngle.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Turret;

const frc::Pose2d TurretController::kDrivetrainToTurretFrame{
    2_in, 0_m, wpi::math::pi * 1_rad};

TurretController::TurretController()
    : ControllerBase("Turret",
                     {ControllerLabel{"Angle", "rad"},
                      ControllerLabel{"Angular velocity", "rad/s"}},
                     {ControllerLabel{"Voltage", "V"}},
                     {ControllerLabel{"Angle", "rad"}}) {
    Reset();
}

void TurretController::SetGoal(
    units::radian_t angleGoal,
    units::radians_per_second_t angularVelocityGoal) {
    m_goal = {NormalizeAngle(angleGoal), angularVelocityGoal};
}

void TurretController::SetReferences(units::radian_t angle,
                                     units::radians_per_second_t velocity) {
    m_nextR << angle.to<double>(), velocity.to<double>();
}

bool TurretController::AtReferences() const { return m_atReferences; }

bool TurretController::AtGoal() const {
    frc::TrapezoidProfile<units::radians>::State ref{
        units::radian_t{m_nextR(0)}, units::radians_per_second_t{m_nextR(1)}};
    return m_goal == ref && m_atReferences;
}

void TurretController::SetDrivetrainStates(
    const Eigen::Matrix<double, 10, 1>& x) {
    using State = DrivetrainController::State;

    m_drivetrainNextPoseInGlobal =
        frc::Pose2d(units::meter_t{x(State::kX)}, units::meter_t{x(State::kY)},
                    units::radian_t{x(State::kHeading)});
    m_drivetrainLeftVelocity =
        units::meters_per_second_t{x(State::kLeftVelocity)};
    m_drivetrainRightVelocity =
        units::meters_per_second_t{x(State::kRightVelocity)};
}

const Eigen::Matrix<double, 2, 1>& TurretController::GetReferences() const {
    return m_r;
}

const Eigen::Matrix<double, 2, 1>& TurretController::GetStates() const {
    return m_observer.Xhat();
}

void TurretController::Reset() {
    m_observer.Reset();
    m_ff.Reset(Eigen::Matrix<double, 2, 1>::Zero());
    m_r.setZero();
    m_nextR.setZero();
}

Eigen::Matrix<double, 1, 1> TurretController::Update(
    const Eigen::Matrix<double, 1, 1>& y, units::second_t dt) {
    m_observer.Correct(GetInputs(), y);

    Eigen::Matrix<double, 1, 1> u;

    // Calculate next drivetrain and turret pose in global frame
    frc::Transform2d drivetrainToTurretFrame{
        frc::Pose2d(), frc::Pose2d(kDrivetrainToTurretFrame.Translation(),
                                   m_drivetrainNextPoseInGlobal.Rotation())};
    m_turretNextPoseInGlobal =
        m_drivetrainNextPoseInGlobal.TransformBy(drivetrainToTurretFrame);

    // Find angle reference for this timestep
    auto turretHeadingForTargetInGlobal =
        CalculateHeading(ToVector2d(targetPoseInGlobal.Translation()),
                         ToVector2d(m_turretNextPoseInGlobal.Translation()));
    auto turretDesiredHeadingInDrivetrain =
        turretHeadingForTargetInGlobal -
        m_drivetrainNextPoseInGlobal.Rotation().Radians();
    auto turretDesiredHeadingInTurret =
        turretDesiredHeadingInDrivetrain -
        kDrivetrainToTurretFrame.Rotation().Radians();

    // Find angular velocity reference for this timestep
    units::meters_per_second_t v =
        (m_drivetrainLeftVelocity + m_drivetrainRightVelocity) / 2.0;
    units::meters_per_second_t v_x =
        v * m_drivetrainNextPoseInGlobal.Rotation().Cos();
    units::meters_per_second_t v_y =
        v * m_drivetrainNextPoseInGlobal.Rotation().Sin();

    auto turretDesiredAngularVelocity = CalculateAngularVelocity(
        ToVector2d(v_x, v_y),
        ToVector2d(targetPoseInGlobal.Translation() -
                   m_turretNextPoseInGlobal.Translation()));

    SetGoal(turretDesiredHeadingInTurret, turretDesiredAngularVelocity);

    // Calculate profiled references to the goal
    frc::TrapezoidProfile<units::radians>::State references = {
        units::radian_t{m_nextR(State::kAngle)},
        units::radians_per_second_t{m_nextR(State::kAngularVelocity)}};
    frc::TrapezoidProfile<units::radians> profile{m_constraints, m_goal,
                                                  references};
    auto profiledReference = profile.Calculate(Constants::kDt);
    SetReferences(profiledReference.position, profiledReference.velocity);

    Eigen::Matrix<double, 2, 1> error = m_r - m_observer.Xhat();
    error(0) = NormalizeAngle(error(0));

    if (IsEnabled()) {
        u << m_lqr.K() * error + m_ff.Calculate(m_nextR);

        units::radian_t heading{m_observer.Xhat(State::kAngle)};
        if (heading > kCCWLimit && u(0) > 0.0) {
            u << 0.0;
        } else if (heading < kCWLimit && u(0) < 0.0) {
            u << 0.0;
        }

        u *= 12.0 / frc::RobotController::GetInputVoltage();
        u = frc::NormalizeInputVector<1>(u, 12.0);
    } else {
        u << 0.0;
    }

    m_atReferences =
        units::math::abs(units::radian_t{m_nextR(State::kAngle) -
                                         m_observer.Xhat(State::kAngle)}) <
            kAngleTolerance &&
        units::math::abs(units::radians_per_second_t{
            m_nextR(State::kAngularVelocity) -
            m_observer.Xhat(State::kAngularVelocity)}) <
            kAngularVelocityTolerance;

    m_r = m_nextR;
    m_observer.Predict(u, dt);

    return u;
}

const frc::LinearSystem<2, 1, 1>& TurretController::GetPlant() const {
    return m_plant;
}

frc::Pose2d TurretController::GetNextPose() const {
    return m_turretNextPoseInGlobal;
}

units::radian_t TurretController::CalculateHeading(Eigen::Vector2d target,
                                                   Eigen::Vector2d turret) {
    return units::math::atan2(units::meter_t{target(1) - turret(1)},
                              units::meter_t{target(0) - turret(0)});
}

units::radians_per_second_t TurretController::CalculateAngularVelocity(
    Eigen::Vector2d v, Eigen::Vector2d r) {
    // We want the angular velocity around the target. We know:
    //
    // 1) velocity vector of the turret in the global frame
    // 2) displacement vector from the target to the turret
    //
    // v = w x r where v is the velocity vector, w is the angular velocity
    // vector, and r is the displacement vector from the center of rotation.
    //
    // |w| = |v_perp| / |r| where v_perp is the component of v perpendicular to
    // the displacement vector.
    //
    // |w| = |v_perp| / |r|             (1)
    // v_perp = v - proj_r(v)           (2)
    // proj_r(v) = v . r / (r . r) * r  (3)
    //
    // |w| = |v_perp| / |r|
    // |w| = |(v - proj_r(v))| / |r|
    // |w| = |(v - v . r / (r . r) * r| / |r|
    return units::radians_per_second_t{(v - v.dot(r) / r.dot(r) * r).norm() /
                                       r.norm()};
}

Eigen::Vector2d TurretController::ToVector2d(frc::Translation2d translation) {
    Eigen::Vector2d result;
    result << translation.X().to<double>(), translation.Y().to<double>();
    return result;
}
