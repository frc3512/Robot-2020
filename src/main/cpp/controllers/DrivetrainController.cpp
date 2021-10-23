// Copyright (c) 2018-2021 FRC Team 3512. All Rights Reserved.

#include "controllers/DrivetrainController.hpp"

#include <algorithm>
#include <cmath>

#include <frc/MathUtil.h>
#include <frc/fmt/Eigen.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVelocitySystemConstraint.h>

#include "controllers/LQR.hpp"

using namespace frc3512;

const frc::LinearSystem<2, 2, 2> DrivetrainController::kPlant{GetPlant()};

DrivetrainController::DrivetrainController() {
    m_A = JacobianX(Eigen::Matrix<double, 7, 1>::Zero());
    m_B = JacobianU(Eigen::Matrix<double, 2, 1>::Zero());

    Eigen::Matrix<double, 7, 1> x = Eigen::Matrix<double, 7, 1>::Zero();
    for (auto velocity = -kMaxV; velocity < kMaxV; velocity += 0.01_mps) {
        x(State::kLeftVelocity) = velocity.value();
        x(State::kRightVelocity) = velocity.value();
        m_table.Insert(velocity, ControllerGainForState(x));
    }

    m_trajectoryTimeElapsed.Start();
}

void DrivetrainController::AddTrajectory(
    const frc::Pose2d& start, const std::vector<frc::Translation2d>& interior,
    const frc::Pose2d& end, const frc::TrajectoryConfig& config) {
    bool hadTrajectory = HaveTrajectory();

    m_trajectory = m_trajectory + frc::TrajectoryGenerator::GenerateTrajectory(
                                      start, interior, end, config);
    m_goal = m_trajectory.States().back().pose;

    // If a trajectory wasn't being tracked until now, reset the timer.
    // Otherwise, let the timer continue on the current trajectory.
    if (!hadTrajectory) {
        m_trajectoryTimeElapsed.Reset();
    }
}

void DrivetrainController::AddTrajectory(
    const std::vector<frc::Pose2d>& waypoints,
    const frc::TrajectoryConfig& config) {
    bool hadTrajectory = HaveTrajectory();

    m_trajectory = m_trajectory + frc::TrajectoryGenerator::GenerateTrajectory(
                                      waypoints, config);
    m_goal = m_trajectory.States().back().pose;

    // If a trajectory wasn't being tracked until now, reset the timer.
    // Otherwise, let the timer continue on the current trajectory.
    if (!hadTrajectory) {
        m_trajectoryTimeElapsed.Reset();
    }
}

bool DrivetrainController::HaveTrajectory() const {
    return m_trajectory.States().size() > 0;
}

void DrivetrainController::AbortTrajectories() {
    m_trajectory = frc::Trajectory{};
}

bool DrivetrainController::AtGoal() const {
    frc::Pose2d ref{units::meter_t{m_r(State::kX)},
                    units::meter_t{m_r(State::kY)},
                    units::radian_t{m_r(State::kHeading)}};

    // Checking the time ensures a trajectory with the same start and end pose
    // will get tracked before AtGoal() returns true
    return m_goal == ref &&
           m_trajectoryTimeElapsed.Get() >= m_trajectory.TotalTime() &&
           m_atReferences;
}

void DrivetrainController::Reset(const frc::Pose2d& initialPose) {
    Eigen::Matrix<double, 7, 1> xHat;
    xHat(0) = initialPose.X().value();
    xHat(1) = initialPose.Y().value();
    xHat(2) = initialPose.Rotation().Radians().value();
    xHat.block<4, 1>(3, 0).setZero();

    m_ff.Reset(xHat);
    m_r = xHat;
    m_nextR = xHat;
    m_goal = initialPose;

    UpdateAtReferences(Eigen::Matrix<double, 5, 1>::Zero());
}

Eigen::Matrix<double, 2, 1> DrivetrainController::Calculate(
    const Eigen::Matrix<double, 7, 1>& x) {
    m_u << 0.0, 0.0;

    if (HaveTrajectory()) {
        frc::Trajectory::State ref =
            m_trajectory.Sample(m_trajectoryTimeElapsed.Get());
        auto [vlRef, vrRef] =
            ToWheelVelocities(ref.velocity, ref.curvature, kWidth);

        m_nextR << ref.pose.X().value(), ref.pose.Y().value(),
            ref.pose.Rotation().Radians().value(), vlRef.value(), vrRef.value(),
            0, 0;

        Eigen::Matrix<double, 2, 1> u_fb = Controller(x, m_r);
        u_fb = frc::NormalizeInputVector<2>(u_fb, 12.0);
        m_u = u_fb + m_ff.Calculate(m_nextR);
        m_u = frc::NormalizeInputVector<2>(m_u, 12.0);

        Eigen::Matrix<double, 5, 1> error =
            m_nextR.block<5, 1>(0, 0) - x.block<5, 1>(0, 0);
        error(State::kHeading) =
            frc::AngleModulus(units::radian_t{error(State::kHeading)}).value();
        UpdateAtReferences(error);

        m_r = m_nextR;
    }

    if (AtGoal() && HaveTrajectory()) {
        m_trajectory = frc::Trajectory{};
        m_trajectoryTimeElapsed.Reset();
    }

    return m_u;
}

frc::LinearSystem<2, 2, 2> DrivetrainController::GetPlant() {
    return frc::LinearSystemId::IdentifyDrivetrainSystem(kLinearV, kLinearA,
                                                         kAngularV, kAngularA);
}

frc::TrajectoryConfig DrivetrainController::MakeTrajectoryConfig() {
    return MakeTrajectoryConfig(0_mps, 0_mps);
}

frc::TrajectoryConfig DrivetrainController::MakeTrajectoryConfig(
    units::meters_per_second_t startVelocity,
    units::meters_per_second_t endVelocity) {
    frc::TrajectoryConfig config{kMaxV, kMaxA - 14.5_mps_sq};

    config.AddConstraint(frc::DifferentialDriveVelocitySystemConstraint{
        kPlant, frc::DifferentialDriveKinematics{kWidth}, 8_V});

    // Slows drivetrain down on curves to avoid understeer that introduces
    // odometry errors
    config.AddConstraint(frc::CentripetalAccelerationConstraint{3_mps_sq});

    config.SetStartVelocity(startVelocity);
    config.SetEndVelocity(endVelocity);

    return config;
}

Eigen::Matrix<double, 2, 5> DrivetrainController::ControllerGainForState(
    const Eigen::Matrix<double, 7, 1>& x) {
    // The DARE is ill-conditioned if the velocity is close to zero, so don't
    // let the system stop.
    double velocity =
        (x(State::kLeftVelocity) + x(State::kRightVelocity)) / 2.0;
    if (std::abs(velocity) < 1e-4) {
        return Eigen::Matrix<double, 2, 5>::Zero();
    }

    m_A(State::kY, State::kHeading) = velocity;
    return LQR<5, 2>(m_A, m_B, m_Q, m_R, Constants::kControllerPeriod);
}

Eigen::Matrix<double, 2, 1> DrivetrainController::Controller(
    const Eigen::Matrix<double, 7, 1>& x,
    const Eigen::Matrix<double, 7, 1>& r) {
    // This implements the linear time-varying differential drive controller in
    // theorem 9.6.3 of https://tavsys.net/controls-in-frc.
    units::meters_per_second_t velocity{
        (x(State::kLeftVelocity) + x(State::kRightVelocity)) / 2.0};
    const auto& K = m_table[velocity];

    Eigen::Matrix<double, 5, 5> inRobotFrame =
        Eigen::Matrix<double, 5, 5>::Identity();
    inRobotFrame(0, 0) = std::cos(x(State::kHeading));
    inRobotFrame(0, 1) = std::sin(x(State::kHeading));
    inRobotFrame(1, 0) = -std::sin(x(State::kHeading));
    inRobotFrame(1, 1) = std::cos(x(State::kHeading));

    Eigen::Matrix<double, 5, 1> error =
        r.block<5, 1>(0, 0) - x.block<5, 1>(0, 0);
    error(State::kHeading) =
        frc::AngleModulus(units::radian_t{error(State::kHeading)}).value();
    return K * inRobotFrame * error;
}

Eigen::Matrix<double, 7, 1> DrivetrainController::Dynamics(
    const Eigen::Matrix<double, 7, 1>& x,
    const Eigen::Matrix<double, 2, 1>& u) {
    Eigen::Matrix<double, 4, 2> B;
    B.block<2, 2>(0, 0) = kPlant.B();
    B.block<2, 2>(2, 0).setZero();
    Eigen::Matrix<double, 4, 4> A;
    A.block<2, 2>(0, 0) = kPlant.A();
    A.block<2, 2>(2, 0).setIdentity();
    A.block<4, 2>(0, 2).setZero();

    double v = (x(State::kLeftVelocity) + x(State::kRightVelocity)) / 2.0;

    Eigen::Matrix<double, 7, 1> xdot;
    xdot(0) = v * std::cos(x(State::kHeading));
    xdot(1) = v * std::sin(x(State::kHeading));
    xdot(2) =
        ((x(State::kRightVelocity) - x(State::kLeftVelocity)) / kWidth).value();
    xdot.block<4, 1>(3, 0) = A * x.block<4, 1>(3, 0) + B * u;
    return xdot;
}

Eigen::Matrix<double, 5, 5> DrivetrainController::JacobianX(
    const Eigen::Matrix<double, 7, 1>& x) {
    // clang-format off
    return frc::MakeMatrix<5, 5>(
        0.0, 0.0, 0.0, 0.5, 0.5,
        0.0, 0.0, (x(State::kLeftVelocity) + x(State::kRightVelocity)) / 2.0, 0.0, 0.0,
        0.0, 0.0, 0.0, -1.0 / kWidth.value(), 1.0 / kWidth.value(),
        0.0, 0.0, 0.0, kPlant.A(0, 0), kPlant.A(0, 1),
        0.0, 0.0, 0.0, kPlant.A(1, 0), kPlant.A(1, 1));
    // clang-format on
}

Eigen::Matrix<double, 5, 2> DrivetrainController::JacobianU(
    const Eigen::Matrix<double, 2, 1>& u) {
    // clang-format off
    return frc::MakeMatrix<5, 2>(
        0.0, 0.0,
        0.0, 0.0,
        0.0, 0.0,
        kPlant.B(0, 0), kPlant.B(0, 1),
        kPlant.B(1, 0), kPlant.B(1, 1));
    // clang-format on
}

Eigen::Matrix<double, 5, 1> DrivetrainController::LocalMeasurementModel(
    const Eigen::Matrix<double, 7, 1>& x,
    const Eigen::Matrix<double, 2, 1>& u) {
    auto plant = frc::LinearSystemId::IdentifyDrivetrainSystem(
        kLinearV, kLinearA, kAngularV, kAngularA);
    Eigen::Matrix<double, 2, 1> xdot =
        plant.A() * x.block<2, 1>(State::kLeftVelocity, 0) + plant.B() * u;

    Eigen::Matrix<double, 5, 1> y;
    y << x(State::kHeading), x(State::kLeftPosition), x(State::kRightPosition),
        (xdot(0) + xdot(1)) / 2.0,
        (x(State::kRightVelocity) * x(State::kRightVelocity) -
         x(State::kLeftVelocity) * x(State::kLeftVelocity)) /
            (2.0 * kWidth.value());
    return y;
}

Eigen::Matrix<double, 2, 1> DrivetrainController::GlobalMeasurementModel(
    const Eigen::Matrix<double, 7, 1>& x,
    const Eigen::Matrix<double, 2, 1>& u) {
    static_cast<void>(u);

    Eigen::Matrix<double, 2, 1> y;
    y << x(State::kX), x(State::kY);
    return y;
}

void DrivetrainController::UpdateAtReferences(
    const Eigen::Matrix<double, 5, 1>& error) {
    m_atReferences = std::abs(error(0)) < kPositionTolerance &&
                     std::abs(error(1)) < kPositionTolerance &&
                     std::abs(error(2)) < kAngleTolerance &&
                     std::abs(error(3)) < kVelocityTolerance &&
                     std::abs(error(4)) < kVelocityTolerance;
}
