// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#include "controllers/TurretController.hpp"

#include <frc/RobotController.h>
#include <units/math.h>

#include "controllers/DrivetrainController.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Turret;

const frc::Pose2d TurretController::kDrivetrainToTurretFrame{
    2_in, 0_m, wpi::math::pi * 1_rad};

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

TurretController::TurretController()
    : ControllerBase("Turret",
                     {ControllerLabel{"Angle", "rad"},
                      ControllerLabel{"Angular velocity", "rad/s"}},
                     {ControllerLabel{"Voltage", "V"}},
                     {ControllerLabel{"Angle", "rad"}}) {
    Reset(0_rad);
}

void TurretController::SetGoal(units::radian_t angle,
                               units::radians_per_second_t angularVelocity) {
    m_goal = {units::math::NormalizeAngle(angle), angularVelocity};
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
    const Eigen::Matrix<double, 7, 1>& x) {
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

void TurretController::Reset(units::radian_t initialHeading) {
    Eigen::Matrix<double, 2, 1> xHat;
    xHat << initialHeading.to<double>(), 0.0;

    m_observer.Reset();
    m_observer.SetXhat(xHat);
    m_ff.Reset(xHat);
    m_r = xHat;
    m_nextR = xHat;
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
        CalculateHeading(targetPoseInGlobal.Translation(),
                         m_turretNextPoseInGlobal.Translation());
    auto turretDesiredHeadingInDrivetrain =
        turretHeadingForTargetInGlobal -
        m_drivetrainNextPoseInGlobal.Rotation().Radians();
    auto turretDesiredHeadingInTurret =
        turretDesiredHeadingInDrivetrain -
        kDrivetrainToTurretFrame.Rotation().Radians();

    // Find angular velocity reference for this timestep
    units::meters_per_second_t v =
        (m_drivetrainLeftVelocity + m_drivetrainRightVelocity) / 2.0;
    frc::Velocity2d drivetrainVelocity{v,
                                       m_drivetrainNextPoseInGlobal.Rotation()};

    auto turretDesiredAngularVelocity = CalculateAngularVelocity(
        drivetrainVelocity, targetPoseInGlobal.Translation() -
                                m_turretNextPoseInGlobal.Translation());

    SetGoal(turretDesiredHeadingInTurret, turretDesiredAngularVelocity);

    // Calculate profiled references to the goal
    frc::TrapezoidProfile<units::radians>::State references = {
        units::radian_t{m_nextR(State::kAngle)},
        units::radians_per_second_t{m_nextR(State::kAngularVelocity)}};
    frc::TrapezoidProfile<units::radians> profile{m_constraints, m_goal,
                                                  references};
    auto profiledReference = profile.Calculate(Constants::kDt);
    SetReferences(profiledReference.position, profiledReference.velocity);

    if (IsClosedLoop()) {
        u << m_lqr.K() * (m_r - m_observer.Xhat()) + m_ff.Calculate(m_nextR);

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

units::radian_t TurretController::CalculateHeading(frc::Translation2d target,
                                                   frc::Translation2d turret) {
    units::meters_per_second_t drivetrainSpeed{
        (m_drivetrainLeftVelocity + m_drivetrainRightVelocity) / 2.0};
    units::radian_t drivetrainHeading =
        m_drivetrainNextPoseInGlobal.Rotation().Radians();

    frc::Velocity2d drivetrainVelocity{drivetrainSpeed,
                                       frc::Rotation2d{drivetrainHeading}};

    return units::math::atan2(target.Y() - turret.Y(),
                              target.X() - turret.X()) +
           CalculateHeadingAdjustment(turret, drivetrainVelocity,
                                      650_rad_per_s);
}

units::radian_t TurretController::CalculateHeadingAdjustment(
    frc::Translation2d turretTranslationInGlobal,
    frc::Velocity2d drivetrainVelocity,
    units::radians_per_second_t flywheelAngularSpeed) {
    static constexpr auto kFlywheelRadius = 4_in;

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
    // No Translation2d::operator* exists that takes a 1/s and gives a
    // Velocity2d.
    frc::Translation2d v2{units::meter_t{v.X().to<double>()},
                          units::meter_t{v.Y().to<double>()}};

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
    // |w| = |(v - r * (v . r / (r . r))| / |r|
    return units::radians_per_second_t{
        ((v2 - r * (Dot(v2, r) / Dot(r, r))).Norm() / r.Norm()).to<double>()};
}
