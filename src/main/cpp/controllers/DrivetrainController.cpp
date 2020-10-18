// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#include "controllers/DrivetrainController.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#include <frc/RobotController.h>
#include <frc/StateSpaceUtil.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/system/NumericalJacobian.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc/trajectory/constraint/DifferentialDriveVelocitySystemConstraint.h>
#include <units/math.h>
#include <wpi/raw_ostream.h>

using namespace frc3512;
using namespace frc3512::Constants;

const Eigen::Matrix<double, 2, 2> DrivetrainController::kGlobalR =
    frc::MakeCovMatrix(0.05, 0.05);

DrivetrainController::DrivetrainController()
    : ControllerBase("Drivetrain",
                     {ControllerLabel{"X", "m"}, ControllerLabel{"Y", "m"},
                      ControllerLabel{"Heading", "rad"},
                      ControllerLabel{"Left velocity", "m/s"},
                      ControllerLabel{"Right velocity", "m/s"},
                      ControllerLabel{"Left position", "m"},
                      ControllerLabel{"Right position", "m"},
                      ControllerLabel{"Left voltage error", "V"},
                      ControllerLabel{"Right voltage error", "V"},
                      ControllerLabel{"Heading error", "rad"}},
                     {ControllerLabel{"Left voltage", "V"},
                      ControllerLabel{"Right voltage", "V"}},
                     {ControllerLabel{"Heading", "rad"},
                      ControllerLabel{"Left position", "m"},
                      ControllerLabel{"Right position", "m"}}) {
    Reset(frc::Pose2d{0_m, 0_m, 0_rad});

    m_B = frc::NumericalJacobianU<10, 10, 2>(
              Dynamics, Eigen::Matrix<double, 10, 1>::Zero(),
              Eigen::Matrix<double, 2, 1>::Zero())
              .block<5, 2>(0, 0);

    m_timeSinceSetWaypoints.Start();
}

void DrivetrainController::SetWaypoints(
    const frc::Pose2d& start, const std::vector<frc::Translation2d>& interior,
    const frc::Pose2d& end) {
    auto config = MakeTrajectoryConfig();
    SetWaypoints(start, interior, end, config);
}

void DrivetrainController::SetWaypoints(
    const frc::Pose2d& start, const std::vector<frc::Translation2d>& interior,
    const frc::Pose2d& end, frc::TrajectoryConfig& config) {
    m_goal = end;
    m_trajectory = frc::TrajectoryGenerator::GenerateTrajectory(start, interior,
                                                                end, config);
    m_timeSinceSetWaypoints.Reset();
    SetClosedLoop(true);
}

bool DrivetrainController::AtGoal() const {
    frc::Pose2d ref{units::meter_t{m_r(State::kX)},
                    units::meter_t{m_r(State::kY)},
                    units::radian_t{m_r(State::kHeading)}};
    return m_goal == ref && m_atReferences;
}

void DrivetrainController::Predict(const Eigen::Matrix<double, 2, 1>& u,
                                   units::second_t dt) {
    m_observer.Predict(u, dt);
}

void DrivetrainController::CorrectWithGlobalOutputs(units::meter_t x,
                                                    units::meter_t y,
                                                    units::second_t timestamp) {
    Eigen::Matrix<double, 2, 1> globalY;
    globalY << x.to<double>(), y.to<double>();
    m_latencyComp.ApplyPastMeasurement<2>(
        &m_observer, Constants::kDt, globalY,
        [&](const Eigen::Matrix<double, 2, 1>& u,
            const Eigen::Matrix<double, 2, 1>& y) {
            m_observer.Correct<2>(
                u, y, &DrivetrainController::GlobalMeasurementModel, kGlobalR);
        },
        timestamp);
}

const Eigen::Matrix<double, 10, 1>& DrivetrainController::GetReferences()
    const {
    return m_r;
}

const Eigen::Matrix<double, 10, 1>& DrivetrainController::GetStates() const {
    return m_observer.Xhat();
}

void DrivetrainController::Reset(const frc::Pose2d& initialPose) {
    Eigen::Matrix<double, 10, 1> xHat;
    xHat(0) = initialPose.X().to<double>();
    xHat(1) = initialPose.Y().to<double>();
    xHat(2) = initialPose.Rotation().Radians().to<double>();
    xHat.block<7, 1>(3, 0).setZero();

    m_observer.Reset();
    m_observer.SetXhat(xHat);
    m_ff.Reset(xHat);
    m_r = xHat;
    m_nextR = xHat;
}

Eigen::Matrix<double, 2, 1> DrivetrainController::Update(
    const Eigen::Matrix<double, 3, 1>& y, units::second_t dt) {
    m_latencyComp.AddObserverState(m_observer, GetInputs(), y,
                                   frc2::Timer::GetFPGATimestamp());
    m_observer.Correct(GetInputs(), y);

    Eigen::Matrix<double, 2, 1> u;

    if (IsClosedLoop()) {
        frc::Trajectory::State ref;

        // Only sample the trajectory if one was created.
        {
            if (m_trajectory.States().size() != 0) {
                ref = m_trajectory.Sample(m_timeSinceSetWaypoints.Get());
            } else {
                ref.pose = frc::Pose2d(
                    units::meter_t{m_observer.Xhat(State::kX)},
                    units::meter_t{m_observer.Xhat(State::kY)},
                    units::radian_t{m_observer.Xhat(State::kHeading)});
            }
        }
        auto [vlRef, vrRef] =
            ToWheelVelocities(ref.velocity, ref.curvature, kWidth);

        m_nextR << ref.pose.X().to<double>(), ref.pose.Y().to<double>(),
            ref.pose.Rotation().Radians().to<double>(), vlRef.to<double>(),
            vrRef.to<double>(), 0, 0, 0, 0, 0;

        u = Controller(m_observer.Xhat(), m_r) + m_ff.Calculate(m_nextR);
        ScaleCapU(&u);

        Eigen::Matrix<double, 5, 1> error =
            m_r.block<5, 1>(0, 0) - m_observer.Xhat().block<5, 1>(0, 0);
        m_atReferences = std::abs(error(0)) < kPositionTolerance &&
                         std::abs(error(1)) < kPositionTolerance &&
                         std::abs(error(2)) < kAngleTolerance &&
                         std::abs(error(3)) < kVelocityTolerance &&
                         std::abs(error(4)) < kVelocityTolerance;
    } else {
        u << 0.0, 0.0;
    }

    m_r = m_nextR;
    m_observer.Predict(u, dt);

    return u;
}

frc::LinearSystem<2, 2, 2> DrivetrainController::GetPlant() const {
    return frc::LinearSystemId::IdentifyDrivetrainSystem(
        kLinearV.to<double>(), kLinearA.to<double>(), kAngularV.to<double>(),
        kAngularA.to<double>());
}

frc::TrajectoryConfig DrivetrainController::MakeTrajectoryConfig() const {
    frc::TrajectoryConfig config{kMaxV, kMaxA - 16.5_mps_sq};

    auto plant = frc::LinearSystemId::IdentifyDrivetrainSystem(
        kLinearV.to<double>(), kLinearA.to<double>(), kAngularV.to<double>(),
        kAngularA.to<double>());
    frc::DifferentialDriveKinematics kinematics{kWidth};
    frc::DifferentialDriveVelocitySystemConstraint systemConstraint{
        plant, kinematics, 8_V};
    config.AddConstraint(systemConstraint);

    return config;
}

Eigen::Matrix<double, 2, 5> DrivetrainController::ControllerGainForState(
    const Eigen::Matrix<double, 10, 1>& x) {
    // Make the heading zero because the LTV controller controls forward error
    // and cross-track error
    Eigen::Matrix<double, 10, 1> x0 = x;
    x0(State::kHeading) = 0.0;

    // The DARE is ill-conditioned if the velocity is close to zero, so don't
    // let the system stop.
    double velocity =
        (x0(State::kLeftVelocity) + x0(State::kRightVelocity)) / 2.0;
    if (std::abs(velocity) < 1e-4) {
        return Eigen::Matrix<double, 2, 5>::Zero();
    }

    Eigen::Matrix<double, 5, 5> A =
        frc::NumericalJacobianX<10, 10, 2>(Dynamics, x0,
                                           Eigen::Matrix<double, 2, 1>::Zero())
            .block<5, 5>(0, 0);

    return frc::LinearQuadraticRegulator<5, 2>(A, m_B, kControllerQ,
                                               kControllerR, kDt)
        .K();
}

Eigen::Matrix<double, 2, 1> DrivetrainController::Controller(
    const Eigen::Matrix<double, 10, 1>& x,
    const Eigen::Matrix<double, 10, 1>& r) {
    // This implements the linear time-varying differential drive controller in
    // theorem 8.6.4 of https://tavsys.net/controls-in-frc.

    try {
        m_K = ControllerGainForState(x);
    } catch (const std::runtime_error& e) {
        wpi::outs() << e.what() << '\n';

        Eigen::Matrix<double, 10, 1> x0 = x;
        x0(State::kHeading) = 0.0;

        // The DARE is ill-conditioned if the velocity is close to zero, so
        // don't let the system stop.
        double velocity =
            (x0(State::kLeftVelocity) + x0(State::kRightVelocity)) / 2.0;
        if (std::abs(velocity) < 1e-3) {
            x0(State::kLeftVelocity) += 1e-3;
            x0(State::kRightVelocity) += 1e-3;
        }

        Eigen::Matrix<double, 5, 5> A =
            frc::NumericalJacobianX<10, 10, 2>(
                Dynamics, x0, Eigen::Matrix<double, 2, 1>::Zero())
                .block<5, 5>(0, 0);

        Eigen::Matrix<double, 5, 5> discA;
        Eigen::Matrix<double, 5, 2> discB;
        frc::DiscretizeAB<5, 2>(A, m_B, kDt, &discA, &discB);
        std::cout << "A=\n" << discA << '\n';
        std::cout << "B=\n" << discB << '\n';
    }

    Eigen::Matrix<double, 5, 5> inRobotFrame =
        Eigen::Matrix<double, 5, 5>::Identity();
    inRobotFrame(0, 0) = std::cos(x(2));
    inRobotFrame(0, 1) = std::sin(x(2));
    inRobotFrame(1, 0) = -std::sin(x(2));
    inRobotFrame(1, 1) = std::cos(x(2));

    Eigen::Matrix<double, 5, 1> error =
        r.block<5, 1>(0, 0) - x.block<5, 1>(0, 0);
    error(State::kHeading) =
        units::math::NormalizeAngle(units::radian_t{error(State::kHeading)})
            .to<double>();
    return m_K * inRobotFrame * error;
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

    auto plant = frc::LinearSystemId::IdentifyDrivetrainSystem(
        kLinearV.to<double>(), kLinearA.to<double>(), kAngularV.to<double>(),
        kAngularA.to<double>());

    Eigen::Matrix<double, 4, 2> B;
    B.block<2, 2>(0, 0) = plant.B();
    B.block<2, 2>(2, 0).setZero();
    Eigen::Matrix<double, 4, 7> A;
    A.block<2, 2>(0, 0) = plant.A();

    A.block<2, 2>(2, 0).setIdentity();
    A.block<4, 2>(0, 2).setZero();
    A.block<4, 2>(0, 4) = B;
    A.block<4, 1>(0, 6) << 0, 0, 1, -1;

    double v = (x(State::kLeftVelocity) + x(State::kRightVelocity)) / 2.0;

    Eigen::Matrix<double, 10, 1> xdot;
    xdot(0) = v * std::cos(x(State::kHeading));
    xdot(1) = v * std::sin(x(State::kHeading));
    xdot(2) =
        ((x(State::kRightVelocity) - x(State::kLeftVelocity)) / (2.0 * rb))
            .to<double>();
    xdot.block<4, 1>(3, 0) = A * x.block<7, 1>(3, 0) + B * u;
    xdot.block<3, 1>(7, 0).setZero();
    return xdot;
}

Eigen::Matrix<double, 3, 1> DrivetrainController::LocalMeasurementModel(
    const Eigen::Matrix<double, 10, 1>& x,
    const Eigen::Matrix<double, 2, 1>& u) {
    static_cast<void>(u);

    Eigen::Matrix<double, 3, 1> y;
    y << x(State::kHeading), x(State::kLeftPosition), x(State::kRightPosition);
    return y;
}

Eigen::Matrix<double, 2, 1> DrivetrainController::GlobalMeasurementModel(
    const Eigen::Matrix<double, 10, 1>& x,
    const Eigen::Matrix<double, 2, 1>& u) {
    static_cast<void>(u);

    Eigen::Matrix<double, 2, 1> y;
    y << x(State::kX), x(State::kY);
    return y;
}

void DrivetrainController::ScaleCapU(Eigen::Matrix<double, 2, 1>* u) {
    double norm = u->lpNorm<Eigen::Infinity>();
    if (norm > 12.0) {
        *u *= 12.0 / norm;
    }
}
