// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "controllers/TurretController.hpp"

#include <frc/MathUtil.h>
#include <frc/RobotController.h>
#include <frc/system/plant/LinearSystemId.h>

#include "TargetModel.hpp"
#include "controllers/DrivetrainController.hpp"

using namespace frc3512;

const frc::Pose2d TurretController::kDrivetrainToTurretFrame{
    2_in, 0_m, wpi::numbers::pi * 1_rad};

template <typename Vector1, typename Vector2>
auto Dot(const Vector1& a, const Vector2& b) -> decltype(auto) {
    // (a_x i + a_y j) . (b_x i + b_y j)
    // = a_x b_x + a_y b_y
    return a.X() * b.X() + a.Y() * b.Y();
}

template <typename Vector1, typename Vector2>
auto Cross(const Vector1& a, const Vector2& b) -> decltype(auto) {
    // (a_x i + a_y j) x (b_x i + b_y j)
    // = a_x b_y - a_y b_x
    return a.X() * b.Y() - a.Y() * b.X();
}

TurretController::TurretController() { Reset(0_rad); }

void TurretController::SetGoal(units::radian_t angle,
                               units::radians_per_second_t angularVelocity) {
    m_goal = {frc::AngleModulus(angle), angularVelocity};
}

void TurretController::SetReferences(units::radian_t angle,
                                     units::radians_per_second_t velocity) {
    m_nextR = Eigen::Vector<double, 2>{angle.value(), velocity.value()};
}

bool TurretController::AtReferences() const { return m_atReferences; }

bool TurretController::AtGoal() const {
    frc::TrapezoidProfile<units::radian>::State ref{
        units::radian_t{m_nextR(0)}, units::radians_per_second_t{m_nextR(1)}};
    return m_goal == ref && m_atReferences;
}

void TurretController::SetDrivetrainStates(const Eigen::Vector<double, 7>& x) {
    using State = DrivetrainController::State;

    m_drivetrainNextPoseInGlobal =
        frc::Pose2d(units::meter_t{x(State::kX)}, units::meter_t{x(State::kY)},
                    units::radian_t{x(State::kHeading)});
    m_drivetrainLeftVelocity =
        units::meters_per_second_t{x(State::kLeftVelocity)};
    m_drivetrainRightVelocity =
        units::meters_per_second_t{x(State::kRightVelocity)};
}

void TurretController::SetFlywheelReferences(units::radians_per_second_t r) {
    m_flywheelAngularVelocityRef = r;
}

void TurretController::SetVisionMeasurements(units::radian_t yaw,
                                             units::second_t timestamp) {
    m_visionYaw = yaw;
    m_timestamp = timestamp;
}

void TurretController::SetControlMode(ControlMode mode) {
    m_controlMode = mode;
}

TurretController::ControlMode TurretController::GetControlMode() const {
    return m_controlMode;
}

void TurretController::Reset(units::radian_t initialHeading) {
    Eigen::Vector<double, 2> xHat{initialHeading.value(), 0.0};

    m_ff.Reset(xHat);
    m_r = xHat;
    m_nextR = xHat;

    UpdateAtReferences(Eigen::Vector<double, 2>::Zero());
}

Eigen::Vector<double, 1> TurretController::Calculate(
    const Eigen::Vector<double, 2>& x) {
    if (m_controlMode == ControlMode::kClosedLoop ||
        m_controlMode == ControlMode::kAutoAim ||
        m_controlMode == ControlMode::kVisionAim) {
        if (m_controlMode == ControlMode::kAutoAim) {
            // Calculate next drivetrain and turret pose in global frame
            auto turretNextPoseInGlobal =
                DrivetrainToTurretInGlobal(m_drivetrainNextPoseInGlobal);

            // Find angle reference for this timestep
            auto turretHeadingForTargetInGlobal = CalculateHeading(
                TargetModel::kTargetPoseInGlobal.Translation() +
                    TargetModel::kOffset,
                turretNextPoseInGlobal.Translation());
            auto turretDesiredHeadingInDrivetrain =
                turretHeadingForTargetInGlobal -
                m_drivetrainNextPoseInGlobal.Rotation().Radians();
            auto turretDesiredHeadingInTurret =
                turretDesiredHeadingInDrivetrain -
                kDrivetrainToTurretFrame.Rotation().Radians();

            // Find angular velocity reference for this timestep
            units::meters_per_second_t drivetrainV =
                (m_drivetrainLeftVelocity + m_drivetrainRightVelocity) / 2.0;
            units::radians_per_second_t drivetrainW =
                (m_drivetrainRightVelocity - m_drivetrainLeftVelocity) /
                DrivetrainController::kWidth * 1_rad;
            frc::Velocity2d drivetrainVelocityInGlobal{
                drivetrainV, m_drivetrainNextPoseInGlobal.Rotation()};

            // The desired turret angular velocity is the turret's angular
            // velocity relative to the target minus the drivetrain's angular
            // velocity relative to the field.
            auto turretDesiredAngularVelocity =
                CalculateAngularVelocity(
                    drivetrainVelocityInGlobal,
                    turretNextPoseInGlobal.Translation() -
                        (TargetModel::kTargetPoseInGlobal.Translation() +
                         TargetModel::kOffset)) -
                drivetrainW;

            // Auto-aiming doesn't need a trapezoid profile because the
            // drivetrain pose already profiles the heading. We still set the
            // trapezoid profile goal to the reference so AtGoal() works with
            // auto-aiming.
            SetGoal(turretDesiredHeadingInTurret, turretDesiredAngularVelocity);
            SetReferences(m_goal.position, m_goal.velocity);
        } else if (m_controlMode == ControlMode::kClosedLoop ||
                   m_controlMode == ControlMode::kVisionAim) {
            if (m_controlMode == ControlMode::kVisionAim) {
                SetGoal(-m_visionYaw + units::radian_t{x(State::kAngle)},
                        0_rad_per_s);
                SetControlMode(ControlMode::kClosedLoop);
            }

            // Calculate profiled references to the goal
            frc::TrapezoidProfile<units::radian>::State references = {
                units::radian_t{m_nextR(State::kAngle)},
                units::radians_per_second_t{m_nextR(State::kAngularVelocity)}};
            frc::TrapezoidProfile<units::radian> profile{m_constraints, m_goal,
                                                         references};
            auto profiledReference =
                profile.Calculate(Constants::kControllerPeriod);
            SetReferences(profiledReference.position,
                          profiledReference.velocity);
        }

        m_u = m_lqr.K() * (m_r - x) + m_ff.Calculate(m_nextR);

        units::radian_t heading{x(State::kAngle)};
        if (heading > kCCWLimit && m_u(Input::kVoltage) > 0.0) {
            m_u = Eigen::Vector<double, 1>::Zero();
        } else if (heading < kCWLimit && m_u(Input::kVoltage) < 0.0) {
            m_u = Eigen::Vector<double, 1>::Zero();
        }

        m_u = frc::DesaturateInputVector<1>(m_u, 12.0);

        UpdateAtReferences(m_nextR - x);
    } else {
        m_u = Eigen::Vector<double, 1>::Zero();
    }

    m_r = m_nextR;

    return m_u;
}

frc::LinearSystem<2, 1, 1> TurretController::GetPlant() {
    return frc::LinearSystemId::IdentifyPositionSystem<units::radian>(kV, kA);
}

units::radian_t TurretController::CalculateHeading(
    frc::Translation2d targetInGlobal, frc::Translation2d turretInGlobal) {
    units::meters_per_second_t drivetrainSpeed{
        (m_drivetrainLeftVelocity + m_drivetrainRightVelocity) / 2.0};
    units::radian_t drivetrainHeading =
        m_drivetrainNextPoseInGlobal.Rotation().Radians();

    frc::Velocity2d drivetrainVelocity{drivetrainSpeed,
                                       frc::Rotation2d{drivetrainHeading}};

    // Subtract 50 rad/s from the nominal flywheel angular velocity because
    // that's roughly how much the flywheel slows down before the ball exits the
    // shooter.
    auto adjustment = CalculateHeadingAdjustment(
        turretInGlobal, drivetrainVelocity,
        units::math::max(0_rad_per_s,
                         m_flywheelAngularVelocityRef - 50_rad_per_s));

    // TODO: Heading adjustment isn't included in angular velocity reference
    // calculation
    return units::math::atan2(targetInGlobal.Y() - turretInGlobal.Y(),
                              targetInGlobal.X() - turretInGlobal.X()) +
           adjustment;
}

units::radian_t TurretController::CalculateHeadingAdjustment(
    frc::Translation2d turretTranslationInGlobal,
    frc::Velocity2d drivetrainVelocity,
    units::radians_per_second_t flywheelAngularSpeed) {
    static constexpr auto kFlywheelRadius = 4_in;

    // If flywheel isn't spinning, don't apply an adjustment because the
    // algorithm below will divide by zero (the heading of a zero vector is
    // undefined).
    if (flywheelAngularSpeed == 0_rad_per_s) {
        return 0_rad;
    }

    // +y
    // ^
    // |
    // |<.
    // |  \ +w
    // |  |
    // ------> +x (out of the shooter)
    //
    //   ___ v_b,top = 0
    //  -   -
    // |  b  | --> v_b,middle
    //  -___-
    //   ___ v_b,bottom = v_f
    //  -   -
    // |  f  |
    //
    // v = ω x r
    // v_f = 0 i + -ω_f k x r_f j
    // v_f i = ω_f r_f i
    // v_f = ω_f r_f
    //
    // v_b,top i = 0 i
    // v_b,top = 0
    // v_b,bottom i = v_f i = ω_f r_f i
    // v_b,bottom = v_f = ω_f r_f
    //
    // v_b,middle = v_b,top + ω_b k x -r_b j
    // v_b,middle i = 0 i + ω_b r_b i
    // v_b,middle = ω_b r_b
    //
    // v_b,top = v_b,bottom + ω_b k x 2r_b j
    // 0 i = v_f i - ω_b 2r_b i
    // 0 = v_f - ω_b 2r_b
    // ω_b = v_f / (2r_b)
    //
    // v_b,middle = (v_f / (2r_b)) r_b
    // v_b,middle = ω_f r_f / 2
    units::meters_per_second_t ballSpeed =
        flywheelAngularSpeed * kFlywheelRadius / 2.0 / 1_rad;

    frc::Translation2d targetPosition{TargetModel::kCenter.X(),
                                      TargetModel::kCenter.Y()};

    auto targetVelocity = -drivetrainVelocity;

    return units::math::asin(
        Cross(targetPosition - turretTranslationInGlobal, targetVelocity) /
        ((targetPosition - turretTranslationInGlobal).Norm() * ballSpeed));
}

units::radians_per_second_t TurretController::CalculateAngularVelocity(
    frc::Velocity2d v, frc::Translation2d r) {
    // We want the angular velocity around the target. This is equivalent to the
    // change in heading to the target over change in time.
    //
    //   ω = d/dt tan⁻¹(r.y / r.x)
    //
    // where r is the vector from the turret to the target.
    //
    //   ω = r x dr/dt / ‖r‖²
    //
    // where dr/dt is the velocity vector of the turret in the global frame.
    return units::radians_per_second_t{Cross(r, v) / Dot(r, r) * 1_rad};
}

frc::Pose2d TurretController::DrivetrainToTurretInGlobal(
    const frc::Pose2d& drivetrainInGlobal) {
    // Calculate next drivetrain and turret pose in global frame
    frc::Transform2d drivetrainToTurretFrame{
        frc::Pose2d{}, frc::Pose2d{kDrivetrainToTurretFrame.Translation(),
                                   drivetrainInGlobal.Rotation()}};
    return drivetrainInGlobal.TransformBy(drivetrainToTurretFrame);
}

void TurretController::UpdateAtReferences(
    const Eigen::Vector<double, 2>& error) {
    m_atReferences = units::math::abs(units::radian_t{error(State::kAngle)}) <
                         kAngleTolerance &&
                     units::math::abs(units::radians_per_second_t{error(
                         State::kAngularVelocity)}) < kAngularVelocityTolerance;
}
