// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#include "controllers/TurretController.hpp"

#include <frc/RobotController.h>

using namespace frc3512;
using namespace frc3512::Constants::Turret;

TurretController::TurretController() { m_y.setZero(); }

void TurretController::Enable() { m_lqr.Enable(); }

void TurretController::Disable() { m_lqr.Disable(); }

void TurretController::SetGoal(
    units::radian_t angleGoal,
    units::radians_per_second_t angularVelocityGoal) {
    m_goal = {angleGoal, angularVelocityGoal};
}

void TurretController::SetReferences(units::radian_t angle,
                                     units::radians_per_second_t velocity) {
    m_nextR << angle.to<double>(), velocity.to<double>();
}

bool TurretController::AtReferences() const { return m_atReferences; }

bool TurretController::AtGoal() const {
    frc::TrapezoidProfile<units::radians>::State ref{
        units::radian_t{m_nextR(0, 0)},
        units::radians_per_second_t{m_nextR(1, 0)}};
    return m_goal == ref && m_atReferences;
}

void TurretController::SetMeasuredOutputs(units::radian_t angle) {
    m_y << angle.to<double>();
}

void TurretController::SetHardLimitOutputs(bool leftLimit, bool rightLimit) {
    m_atLeftLimit = leftLimit;
    m_atRightLimit = rightLimit;
}

void TurretController::SetDrivetrainStatus(
    const Eigen::Matrix<double, 10, 1>& nextXhat) {
    m_drivetrainNextPoseInGlobal = frc::Pose2d(units::meter_t{nextXhat(0, 0)},
                                               units::meter_t{nextXhat(1, 0)},
                                               units::radian_t{nextXhat(2, 0)});
    m_drivetrainNextXhat = nextXhat;
}

frc::Pose2d TurretController::GetNextPose() const {
    return m_turretNextPoseInGlobal;
}

units::volt_t TurretController::ControllerVoltage() const {
    return units::volt_t{m_u(0, 0)};
}

units::radian_t TurretController::EstimatedAngle() const {
    return units::radian_t{m_observer.Xhat(0)};
}

units::radians_per_second_t TurretController::EstimatedAngularVelocity() const {
    return units::radians_per_second_t{m_observer.Xhat(1)};
}

units::radian_t TurretController::AngleReference() {
    return units::radian_t{m_nextR(0, 0)};
}

units::radians_per_second_t TurretController::AngularVelocityReference() {
    return units::radians_per_second_t{m_nextR(1, 0)};
}

units::radian_t TurretController::AngleError() const {
    return units::radian_t{m_nextR(0, 0) - m_observer.Xhat(0)};
}

units::radians_per_second_t TurretController::AngularVelocityError() const {
    return units::radians_per_second_t{m_nextR(1, 0) - m_observer.Xhat(1)};
}

units::radian_t TurretController::CalculateHeading(Eigen::Vector2d target,
                                                   Eigen::Vector2d turret) {
    return units::math::atan2(units::meter_t{target(1) - turret(1)},
                              units::meter_t{target(0) - turret(0)});
}

void TurretController::Update(units::second_t dt, units::second_t elapsedTime) {
    positionLogger.Log(elapsedTime, ControllerVoltage().to<double>(), m_y(0, 0),
                       EstimatedAngle().to<double>(),
                       AngleReference().to<double>());
    velocityLogger.Log(elapsedTime, ControllerVoltage().to<double>(),
                       EstimatedAngularVelocity().to<double>(),
                       AngularVelocityReference().to<double>());

    m_observer.Correct(m_u, m_y);

    // Calculate next drivetrain and turret pose in global frame
    frc::Transform2d turretNextPoseInDrivetrainToGlobal{
        frc::Pose2d(kTx, kTy, 0_rad), m_drivetrainNextPoseInGlobal};
    frc::Pose2d turretNextPoseInLocal{0_m, 0_m, 0_rad};
    m_turretNextPoseInGlobal =
        turretNextPoseInLocal.TransformBy(turretNextPoseInDrivetrainToGlobal);

    // Find angle reference for this timestep
    units::radian_t turretThetaToTargetInGlobal =
        CalculateHeading(ToVector2d(targetPoseInGlobal.Translation()),
                         ToVector2d(m_turretNextPoseInGlobal.Translation()));
    units::radian_t drivetrainNextThetaInGlobal =
        m_drivetrainNextPoseInGlobal.Rotation().Radians();
    units::radian_t turretDesiredHeadingInDrivetrain =
        turretThetaToTargetInGlobal - drivetrainNextThetaInGlobal;

    SetGoal(turretDesiredHeadingInDrivetrain, 0_rad_per_s);

    // Calculate profiled references to the goal
    frc::TrapezoidProfile<units::radians>::State references = {
        AngleReference(), AngularVelocityReference()};
    frc::TrapezoidProfile<units::radians> profile{m_constraints, m_goal,
                                                  references};
    auto profiledReference = profile.Calculate(Constants::kDt);
    SetReferences(profiledReference.position, profiledReference.velocity);

    m_lqr.Update(m_observer.Xhat(), m_nextR);
    if (m_atLeftLimit && m_lqr.U(0) > 0) {
        m_u << 0;
    } else if (m_atRightLimit && m_lqr.U(0) < 0) {
        m_u << 0;
    } else {
        m_u << m_lqr.U(0) * 12.0 / frc::RobotController::GetInputVoltage();
    }

    m_atReferences =
        units::math::abs(AngleError()) < kAngleTolerance &&
        units::math::abs(AngularVelocityError()) < kAngularVelocityTolerance;

    m_observer.Predict(m_u, dt);
}

void TurretController::Reset() {
    m_observer.Reset();
    m_nextR.setZero();
    m_u.setZero();
}

Eigen::Vector2d TurretController::ToVector2d(frc::Translation2d translation) {
    Eigen::Vector2d result;
    result << translation.X().to<double>(), translation.Y().to<double>();
    return result;
}
