// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#include "controllers/DrivetrainController.hpp"

#include <algorithm>
#include <cmath>

#include <Eigen/QR>
#include <frc/RobotController.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/geometry/Pose2d.h>
#include <frc/system/NumericalJacobian.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DrivetrainVelocitySystemConstraint.h>
#include <units/units.h>
#include <wpi/MathExtras.h>

using namespace frc3512;
using namespace frc3512::Constants;
using namespace frc3512::Constants::Drivetrain;

frc::LinearSystem<2, 2, 2> DrivetrainController::m_plant =
    frc::IdentifyDrivetrainSystem(
        Constants::Drivetrain::kLinearV.to<double>(),
        Constants::Drivetrain::kLinearA.to<double>(),
        Constants::Drivetrain::kAngularV.to<double>(),
        Constants::Drivetrain::kAngularA.to<double>());

DrivetrainController::DrivetrainController(const std::array<double, 5>& Qelems,
                                           const std::array<double, 2>& Relems,
                                           units::second_t dt) {
    m_localY.setZero();
    m_globalY.setZero();
    Reset();

    Eigen::Matrix<double, 10, 1> x0;
    x0.setZero();
    x0(State::kLeftVelocity, 0) = 1e-9;
    x0(State::kRightVelocity, 0) = 1e-9;
    Eigen::Matrix<double, 10, 1> x1;
    x1.setZero();
    x1(State::kLeftVelocity, 0) = 1;
    x1(State::kRightVelocity, 0) = 1;
    Eigen::Matrix<double, 2, 1> u0;
    u0.setZero();

    Eigen::Matrix<double, 5, 5> A0 =
        frc::NumericalJacobianX<10, 10, 2>(Dynamics, x0, u0).block<5, 5>(0, 0);
    Eigen::Matrix<double, 5, 5> A1 =
        frc::NumericalJacobianX<10, 10, 2>(Dynamics, x1, u0).block<5, 5>(0, 0);
    m_B =
        frc::NumericalJacobianU<10, 10, 2>(Dynamics, x0, u0).block<5, 2>(0, 0);

    m_K0 = frc::LinearQuadraticRegulator<5, 2>(A0, m_B, Qelems, Relems, dt).K();
    m_K1 = frc::LinearQuadraticRegulator<5, 2>(A1, m_B, Qelems, Relems, dt).K();
}

void DrivetrainController::Enable() { m_isEnabled = true; }

void DrivetrainController::Disable() { m_isEnabled = false; }

bool DrivetrainController::IsEnabled() const { return m_isEnabled; }

void DrivetrainController::SetWaypoints(
    const std::vector<frc::Pose2d>& waypoints) {
    frc::DrivetrainVelocitySystemConstraint constraint{m_plant, kWidth, 8_V};
    frc::TrajectoryConfig config{kMaxV, kMaxA};
    config.AddConstraint(constraint);

    std::lock_guard lock(m_trajectoryMutex);
    m_goal = waypoints.back();
    m_trajectory =
        frc::TrajectoryGenerator::GenerateTrajectory(waypoints, config);
}

bool DrivetrainController::AtGoal() {
    frc::Pose2d ref{units::meter_t{m_r(State::kX, 0)},
                    units::meter_t{m_r(State::kY, 0)},
                    units::radian_t{m_r(State::kHeading, 0)}};
    return m_goal == ref && m_atReferences;
}

void DrivetrainController::SetMeasuredLocalOutputs(
    units::radian_t heading, units::meter_t leftPosition,
    units::meter_t rightPosition) {
    m_localY << heading.to<double>(), leftPosition.to<double>(),
        rightPosition.to<double>();
}

void DrivetrainController::SetMeasuredGlobalOutputs(
    units::meter_t x, units::meter_t y, units::radian_t heading,
    units::meter_t leftPosition, units::meter_t rightPosition,
    units::radians_per_second_t angularVelocity) {
    m_globalY << x.to<double>(), y.to<double>(), heading.to<double>(),
        leftPosition.to<double>(), rightPosition.to<double>(),
        angularVelocity.to<double>();
}

const Eigen::Matrix<double, 5, 1>& DrivetrainController::GetReferences() const {
    return m_nextR;
}

const Eigen::Matrix<double, 10, 1>& DrivetrainController::GetStates() const {
    return m_observer.Xhat();
}

Eigen::Matrix<double, 2, 1> DrivetrainController::GetInputs() const {
    return m_cappedU;
}

const Eigen::Matrix<double, 3, 1>& DrivetrainController::GetOutputs() const {
    return m_localY;
}

Eigen::Matrix<double, 3, 1> DrivetrainController::EstimatedLocalOutputs()
    const {
    return LocalMeasurementModel(m_observer.Xhat(),
                                 Eigen::Matrix<double, 2, 1>::Zero());
}

Eigen::Matrix<double, 6, 1> DrivetrainController::EstimatedGlobalOutputs()
    const {
    return GlobalMeasurementModel(m_observer.Xhat(),
                                  Eigen::Matrix<double, 2, 1>::Zero());
}

void DrivetrainController::Update(units::second_t dt,
                                  units::second_t elapsedTime) {
    frc::Trajectory::State ref;
    {
        std::lock_guard lock(m_trajectoryMutex);
        ref = m_trajectory.Sample(elapsedTime);
    }

    auto [vlRef, vrRef] =
        ToWheelVelocities(ref.velocity, ref.curvature, kWidth);

    positionLogger.Log(elapsedTime, m_observer.Xhat(State::kX),
                       m_observer.Xhat(State::kY),
                       ref.pose.Translation().X().to<double>(),
                       ref.pose.Translation().Y().to<double>(),
                       m_localY(LocalOutput::kLeftPosition, 0),
                       m_localY(LocalOutput::kRightPosition, 0),
                       m_observer.Xhat(State::kLeftPosition),
                       m_observer.Xhat(State::kRightPosition),
                       m_odometer.GetPose().Translation().X().to<double>(),
                       m_odometer.GetPose().Translation().Y().to<double>());

    angleLogger.Log(elapsedTime, m_localY(LocalOutput::kHeading),
                    m_observer.Xhat(State::kHeading),
                    ref.pose.Rotation().Radians().to<double>(),
                    m_observer.Xhat(State::kAngularVelocityError));
    velocityLogger.Log(elapsedTime, m_observer.Xhat(State::kLeftVelocity),
                       m_observer.Xhat(State::kRightVelocity),
                       m_nextR(State::kLeftVelocity, 0),
                       m_nextR(State::kRightVelocity, 0), vlRef.to<double>(),
                       vrRef.to<double>());
    voltageLogger.Log(elapsedTime, m_cappedU(Input::kLeftVoltage, 0),
                      m_cappedU(Input::kRightVoltage, 0),
                      m_observer.Xhat(State::kLeftVoltageError),
                      m_observer.Xhat(State::kRightVoltageError),
                      frc::RobotController::GetInputVoltage());
    errorCovLogger.Log(
        elapsedTime, m_observer.P(State::kX, State::kX),
        m_observer.P(State::kY, State::kY),
        m_observer.P(State::kHeading, State::kHeading),
        m_observer.P(State::kLeftVelocity, State::kLeftVelocity),
        m_observer.P(State::kRightVelocity, State::kRightVelocity),
        m_observer.P(State::kLeftPosition, State::kLeftPosition),
        m_observer.P(State::kRightPosition, State::kRightPosition),
        m_observer.P(State::kLeftVoltageError, State::kLeftVoltageError),
        m_observer.P(State::kRightVoltageError, State::kRightVoltageError),
        m_observer.P(State::kAngularVelocityError,
                     State::kAngularVelocityError));

    m_odometer.Update(units::radian_t{m_localY(LocalOutput::kHeading)},
                      units::meter_t{m_localY(LocalOutput::kLeftPosition)},
                      units::meter_t{m_localY(LocalOutput::kRightPosition)});
    m_observer.Correct(m_cappedU, m_localY);

    m_nextR << ref.pose.Translation().X().to<double>(),
        ref.pose.Translation().Y().to<double>(),
        ref.pose.Rotation().Radians().to<double>(), vlRef.to<double>(),
        vrRef.to<double>();

    // Compute feedforward
    Eigen::Matrix<double, 5, 1> rdot = (m_nextR - m_r) / dt.to<double>();
    Eigen::Matrix<double, 10, 1> rAugmented;
    rAugmented.block<5, 1>(0, 0) = m_r;
    rAugmented.block<5, 1>(5, 0).setZero();
    Eigen::Matrix<double, 2, 1> uff = m_B.householderQr().solve(
        rdot - Dynamics(rAugmented, Eigen::Matrix<double, 2, 1>::Zero())
                   .block<5, 1>(0, 0));

    if (m_isEnabled) {
        m_cappedU = Controller(m_observer.Xhat(), m_nextR) + uff;
    } else {
        m_cappedU = Eigen::Matrix<double, 2, 1>::Zero();
    }
    ScaleCapU(&m_cappedU);

    Eigen::Matrix<double, 5, 1> error =
        m_r - m_observer.Xhat().block<5, 1>(0, 0);
    m_atReferences = std::abs(error(0, 0)) < kPositionTolerance &&
                     std::abs(error(1, 0)) < kPositionTolerance &&
                     std::abs(error(2, 0)) < kAngleTolerance &&
                     std::abs(error(3, 0)) < kVelocityTolerance &&
                     std::abs(error(4, 0)) < kVelocityTolerance;

    m_r = m_nextR;
    m_observer.Predict(m_cappedU, dt);

    if (ref.pose == m_goal) {
        Disable();
    } else {
        Enable();
    }
}

void DrivetrainController::Reset() {
    m_observer.Reset();
    m_r.setZero();
    m_nextR.setZero();
    m_cappedU.setZero();
}

void DrivetrainController::Reset(const frc::Pose2d& initialPose) {
    m_observer.Reset();

    Eigen::Matrix<double, 10, 1> xHat;
    xHat(0, 0) = initialPose.Translation().X().to<double>();
    xHat(1, 0) = initialPose.Translation().Y().to<double>();
    xHat(2, 0) = initialPose.Rotation().Radians().to<double>();
    xHat.block<7, 1>(3, 0).setZero();
    m_observer.SetXhat(xHat);

    m_r.setZero();
    m_nextR.setZero();
    m_cappedU.setZero();
}

Eigen::Matrix<double, 2, 1> DrivetrainController::Controller(
    const Eigen::Matrix<double, 10, 1>& x,
    const Eigen::Matrix<double, 5, 1>& r) {
    double kx = m_K0(0, 0);
    double ky0 = m_K0(0, 1);
    double kvpos0 = m_K0(0, 3);
    double kvneg0 = m_K0(1, 3);
    double ky1 = m_K1(0, 1);
    double ktheta1 = m_K1(0, 2);
    double kvpos1 = m_K1(0, 3);

    double v = (x(State::kLeftVelocity, 0) + x(State::kRightVelocity, 0)) / 2.0;
    double sqrtAbsV = std::sqrt(std::abs(v));

    Eigen::Matrix<double, 2, 5> K;
    K(0, 0) = kx;
    K(0, 1) = (ky0 + (ky1 - ky0) * sqrtAbsV) * wpi::sgn(v);
    K(0, 2) = ktheta1 * sqrtAbsV;
    K(0, 3) = kvpos0 + (kvpos1 - kvpos0) * sqrtAbsV;
    K(0, 4) = kvneg0 - (kvpos1 - kvpos0) * sqrtAbsV;
    K(1, 0) = kx;
    K(1, 1) = -K(0, 1);
    K(1, 2) = -K(0, 2);
    K(1, 3) = K(0, 4);
    K(1, 4) = K(0, 3);

    Eigen::Matrix<double, 2, 1> uError;
    uError << x(State::kLeftVoltageError, 0), x(State::kRightVoltageError, 0);

    Eigen::Matrix<double, 5, 5> inRobotFrame =
        Eigen::Matrix<double, 5, 5>::Identity();
    inRobotFrame(0, 0) = std::cos(x(2, 0));
    inRobotFrame(0, 1) = std::sin(x(2, 0));
    inRobotFrame(1, 0) = -std::sin(x(2, 0));
    inRobotFrame(1, 1) = std::cos(x(2, 0));

    Eigen::Matrix<double, 5, 1> error = r - x.block<5, 1>(0, 0);
    error(State::kHeading, 0) = NormalizeAngle(error(State::kHeading, 0));
    return K * inRobotFrame * error;
}

Eigen::Matrix<double, 10, 1> DrivetrainController::Dynamics(
    const Eigen::Matrix<double, 10, 1>& x,
    const Eigen::Matrix<double, 2, 1>& u) {
    // constexpr auto motors = frc::DCMotor::MiniCIM(3);

    // constexpr units::dimensionless_t Glow = 15.32;  // Low gear ratio
    // constexpr units::dimensionless_t Ghigh = 7.08;  // High gear ratio
    // constexpr auto r = 0.0746125_m;                 // Wheel radius
    // constexpr auto m = 63.503_kg;                   // Robot mass
    // constexpr auto J = 5.6_kg_sq_m;                 // Robot moment of
    // inertia

    // constexpr auto C1 =
    //     -1.0 * Ghigh * Ghigh * motors.Kt / (motors.Kv * motors.R * r * r);
    // constexpr auto C2 = Ghigh * motors.Kt / (motors.R * r);
    // constexpr auto k1 = (1 / m + rb * rb / J);
    // constexpr auto k2 = (1 / m - rb * rb / J);

    Eigen::Matrix<double, 4, 2> B;
    B.block<2, 2>(0, 0) = m_plant.B();
    B.block<2, 2>(2, 0).setZero();
    Eigen::Matrix<double, 4, 7> A;
    A.block<2, 2>(0, 0) = m_plant.A();

    A.block<2, 2>(2, 0).setIdentity();
    A.block<4, 2>(0, 2).setZero();
    A.block<4, 2>(0, 4) = B;
    A.block<4, 1>(0, 6) << 0, 0, 1, -1;

    double v = (x(State::kLeftVelocity, 0) + x(State::kRightVelocity, 0)) / 2.0;

    Eigen::Matrix<double, 10, 1> result;
    result(0, 0) = v * std::cos(x(State::kHeading, 0));
    result(1, 0) = v * std::sin(x(State::kHeading, 0));
    result(2, 0) = ((x(State::kRightVelocity, 0) - x(State::kLeftVelocity, 0)) /
                    (2.0 * rb))
                       .to<double>();
    result.block<4, 1>(3, 0) = A * x.block<7, 1>(3, 0) + B * u;
    result.block<3, 1>(7, 0).setZero();
    return result;
}

Eigen::Matrix<double, 3, 1> DrivetrainController::LocalMeasurementModel(
    const Eigen::Matrix<double, 10, 1>& x,
    const Eigen::Matrix<double, 2, 1>& u) {
    static_cast<void>(u);

    Eigen::Matrix<double, 3, 1> y;
    y << x(State::kHeading, 0), x(State::kLeftPosition, 0),
        x(State::kRightPosition, 0);
    return y;
}

Eigen::Matrix<double, 6, 1> DrivetrainController::GlobalMeasurementModel(
    const Eigen::Matrix<double, 10, 1>& x,
    const Eigen::Matrix<double, 2, 1>& u) {
    static_cast<void>(u);

    Eigen::Matrix<double, 6, 1> y;
    y.block<3, 1>(0, 0) = x.block<3, 1>(0, 0);
    y(3, 0) = x(State::kLeftPosition, 0);
    y(4, 0) = x(State::kRightPosition, 0);
    y(5, 0) = (x(State::kRightVelocity, 0) - x(State::kLeftVelocity, 0)) /
              (2.0 * rb.to<double>());
    return y;
}

void DrivetrainController::ScaleCapU(Eigen::Matrix<double, 2, 1>* u) {
    bool outputCapped =
        std::abs((*u)(0, 0)) > 12.0 || std::abs((*u)(1, 0)) > 12.0;

    if (outputCapped) {
        *u *= 12.0 / u->lpNorm<Eigen::Infinity>();
    }
}
