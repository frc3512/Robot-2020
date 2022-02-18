// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "controllers/DrivetrainController.hpp"

#include <algorithm>
#include <cmath>

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/CentripetalAccelerationConstraint.h>
#include <frc/trajectory/constraint/DifferentialDriveVelocitySystemConstraint.h>

using namespace frc3512;

const frc::LinearSystem<2, 2, 2> DrivetrainController::kPlant{GetPlant()};

DrivetrainController::DrivetrainController() {
    m_fb.SetTolerance(
        frc::Pose2d{{kPositionTolerance, kPositionTolerance}, kAngleTolerance},
        kVelocityTolerance, kVelocityTolerance);
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
           m_fb.AtReference();
}

void DrivetrainController::Reset(const frc::Pose2d& initialPose) {
    Eigen::Vector<double, 7> xHat;
    xHat(0) = initialPose.X().value();
    xHat(1) = initialPose.Y().value();
    xHat(2) = initialPose.Rotation().Radians().value();
    xHat.block<4, 1>(3, 0).setZero();

    m_ff.Reset(xHat);
    m_fb.Calculate(frc::Pose2d{}, 0_mps, 0_mps, frc::Pose2d{}, 0_mps, 0_mps);
    m_r = xHat;
    m_nextR = xHat;
    m_goal = initialPose;
}

Eigen::Vector<double, 2> DrivetrainController::Calculate(
    const Eigen::Vector<double, 7>& x) {
    m_u = Eigen::Vector<double, 2>::Zero();

    if (HaveTrajectory()) {
        frc::Trajectory::State ref =
            m_trajectory.Sample(m_trajectoryTimeElapsed.Get());
        auto [vlRef, vrRef] =
            ToWheelVelocities(ref.velocity, ref.curvature, kWidth);

        m_nextR =
            Eigen::Vector<double, 7>{ref.pose.X().value(),
                                     ref.pose.Y().value(),
                                     ref.pose.Rotation().Radians().value(),
                                     vlRef.value(),
                                     vrRef.value(),
                                     0.0,
                                     0.0};

        frc::Pose2d currentPose{units::meter_t{x(State::kX)},
                                units::meter_t{x(State::kY)},
                                units::radian_t{x(State::kHeading)}};
        Eigen::Vector<double, 2> u_fb = m_fb.Calculate(
            currentPose, units::meters_per_second_t{x(State::kLeftVelocity)},
            units::meters_per_second_t{x(State::kRightVelocity)}, ref);
        u_fb = frc::DesaturateInputVector<2>(u_fb, 12.0);
        m_u = u_fb + m_ff.Calculate(m_nextR);
        m_u = frc::DesaturateInputVector<2>(m_u, 12.0);
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
    frc::TrajectoryConfig config{kMaxV, 2.2_mps_sq};

    config.AddConstraint(frc::DifferentialDriveVelocitySystemConstraint{
        kPlant, frc::DifferentialDriveKinematics{kWidth}, 8_V});

    // Slows drivetrain down on curves to avoid understeer that introduces
    // odometry errors
    config.AddConstraint(frc::CentripetalAccelerationConstraint{3_mps_sq});

    config.SetStartVelocity(startVelocity);
    config.SetEndVelocity(endVelocity);

    return config;
}

Eigen::Vector<double, 7> DrivetrainController::Dynamics(
    const Eigen::Vector<double, 7>& x, const Eigen::Vector<double, 2>& u) {
    Eigen::Matrix<double, 4, 2> B;
    B.block<2, 2>(0, 0) = kPlant.B();
    B.block<2, 2>(2, 0).setZero();
    Eigen::Matrix<double, 4, 4> A;
    A.block<2, 2>(0, 0) = kPlant.A();
    A.block<2, 2>(2, 0).setIdentity();
    A.block<4, 2>(0, 2).setZero();

    double v = (x(State::kLeftVelocity) + x(State::kRightVelocity)) / 2.0;

    Eigen::Vector<double, 7> xdot;
    xdot(0) = v * std::cos(x(State::kHeading));
    xdot(1) = v * std::sin(x(State::kHeading));
    xdot(2) =
        ((x(State::kRightVelocity) - x(State::kLeftVelocity)) / kWidth).value();
    xdot.block<4, 1>(3, 0) = A * x.block<4, 1>(3, 0) + B * u;
    return xdot;
}

Eigen::Vector<double, 5> DrivetrainController::LocalMeasurementModel(
    const Eigen::Vector<double, 7>& x, const Eigen::Vector<double, 2>& u) {
    auto plant = frc::LinearSystemId::IdentifyDrivetrainSystem(
        kLinearV, kLinearA, kAngularV, kAngularA);
    Eigen::Vector<double, 2> xdot =
        plant.A() * x.block<2, 1>(State::kLeftVelocity, 0) + plant.B() * u;

    return Eigen::Vector<double, 5>{
        x(State::kHeading), x(State::kLeftPosition), x(State::kRightPosition),
        (xdot(0) + xdot(1)) / 2.0,
        (x(State::kRightVelocity) * x(State::kRightVelocity) -
         x(State::kLeftVelocity) * x(State::kLeftVelocity)) /
            (2.0 * kWidth.value())};
}

Eigen::Vector<double, 2> DrivetrainController::GlobalMeasurementModel(
    const Eigen::Vector<double, 7>& x, const Eigen::Vector<double, 2>& u) {
    static_cast<void>(u);

    return Eigen::Vector<double, 2>{x(State::kX), x(State::kY)};
}
