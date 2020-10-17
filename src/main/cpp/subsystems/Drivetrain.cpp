// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/Drivetrain.hpp"

#include <algorithm>
#include <cmath>

#include <frc/Joystick.h>
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/StateSpaceUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/acceleration.h>
#include <units/math.h>

#include "CANSparkMaxUtil.hpp"
#include "CurvatureDrive.hpp"
#include "EigenFormat.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Robot;

const Eigen::Matrix<double, 2, 2> Drivetrain::kGlobalR =
    frc::MakeCovMatrix(0.05, 0.05);

Drivetrain::Drivetrain()
    : ControlledSubsystemBase(
          "Drivetrain",
          {ControllerLabel{"X", "m"}, ControllerLabel{"Y", "m"},
           ControllerLabel{"Heading", "rad"},
           ControllerLabel{"Left velocity", "m/s"},
           ControllerLabel{"Right velocity", "m/s"},
           ControllerLabel{"Left position", "m"},
           ControllerLabel{"Right position", "m"}},
          {ControllerLabel{"Left voltage", "V"},
           ControllerLabel{"Right voltage", "V"}},
          {ControllerLabel{"Heading", "rad"},
           ControllerLabel{"Left position", "m"},
           ControllerLabel{"Right position", "m"},
           ControllerLabel{"Acceleration", "m/s^2"}}) {
    SetCANSparkMaxBusUsage(m_leftLeader, Usage::kMinimal);
    SetCANSparkMaxBusUsage(m_leftFollower, Usage::kMinimal);
    SetCANSparkMaxBusUsage(m_rightLeader, Usage::kMinimal);
    SetCANSparkMaxBusUsage(m_rightFollower, Usage::kMinimal);

    m_leftLeader.SetSmartCurrentLimit(60);
    m_leftFollower.SetSmartCurrentLimit(60);
    m_rightLeader.SetSmartCurrentLimit(60);
    m_rightFollower.SetSmartCurrentLimit(60);

    // Ensures CANSparkMax::Get() returns an initialized value
    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);

    m_leftGrbx.SetInverted(true);

    m_leftEncoder.SetSamplesToAverage(10);
    m_rightEncoder.SetSamplesToAverage(10);

    m_leftEncoder.SetReverseDirection(false);
    m_rightEncoder.SetReverseDirection(true);
    m_leftEncoder.SetDistancePerPulse(DrivetrainController::kDpP);
    m_rightEncoder.SetDistancePerPulse(DrivetrainController::kDpP);

    // Reset the pose estimate to the field's bottom-left corner with the turret
    // facing in the target's general direction. This is relatively close to the
    // robot's testing configuration, so the turret won't hit the soft limits.
    Reset(frc::Pose2d{0_m, 0_m, units::radian_t{wpi::math::pi}});

    frc::SmartDashboard::PutData(&m_field);
}

frc::Pose2d Drivetrain::GetPose() const {
    const auto& x = m_observer.Xhat();
    return frc::Pose2d{
        units::meter_t{x(DrivetrainController::State::kX)},
        units::meter_t{x(DrivetrainController::State::kY)},
        units::radian_t{x(DrivetrainController::State::kHeading)}};
}

units::radian_t Drivetrain::GetAngle() const {
    return units::degree_t{m_imu.GetAngle()} + m_headingOffset;
}

units::meter_t Drivetrain::GetLeftPosition() const {
    return units::meter_t{m_leftEncoder.GetDistance()};
}

units::meter_t Drivetrain::GetRightPosition() const {
    return units::meter_t{m_rightEncoder.GetDistance()};
}

units::meters_per_second_t Drivetrain::GetLeftVelocity() const {
    return units::meters_per_second_t{m_leftEncoder.GetRate()};
}

units::meters_per_second_t Drivetrain::GetRightVelocity() const {
    return units::meters_per_second_t{m_rightEncoder.GetRate()};
}

units::meters_per_second_squared_t Drivetrain::GetAcceleration() const {
    return m_imu.GetAccelInstantX() * 9.8_mps_sq;
}

void Drivetrain::Reset(const frc::Pose2d& initialPose) {
    Reset(initialPose, 0_mps, 0_mps);
}

void Drivetrain::Reset(const frc::Pose2d& initialPose,
                       units::meters_per_second_t leftVelocity,
                       units::meters_per_second_t rightVelocity) {
    using State = frc::sim::DifferentialDrivetrainSim::State;

    m_observer.Reset();
    m_controller.Reset(initialPose, leftVelocity, rightVelocity);
    m_u = Eigen::Matrix<double, 2, 1>::Zero();
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
    m_imu.Reset();
    m_headingOffset = initialPose.Rotation().Radians();

    Eigen::Matrix<double, 7, 1> xHat;
    xHat(State::kX) = initialPose.X().to<double>();
    xHat(State::kY) = initialPose.Y().to<double>();
    xHat(State::kHeading) = initialPose.Rotation().Radians().to<double>();
    xHat(State::kLeftVelocity) = leftVelocity.to<double>();
    xHat(State::kRightVelocity) = rightVelocity.to<double>();
    xHat.block<2, 1>(5, 0).setZero();
    m_observer.SetXhat(xHat);

    if constexpr (frc::RobotBase::IsSimulation()) {
        m_drivetrainSim.SetState(xHat);
        m_field.SetRobotPose(initialPose);
    }
}

void Drivetrain::CorrectWithGlobalOutputs(units::meter_t x, units::meter_t y,
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

void Drivetrain::ControllerPeriodic() {
    UpdateDt();

    m_observer.Predict(m_u, GetDt());

    Eigen::Matrix<double, 4, 1> y;
    y << frc::AngleModulus(GetAngle()).to<double>(),
        GetLeftPosition().to<double>(), GetRightPosition().to<double>(),
        GetAcceleration().to<double>();
    m_latencyComp.AddObserverState(m_observer, m_controller.GetInputs(), y,
                                   frc2::Timer::GetFPGATimestamp());
    m_observer.Correct(m_controller.GetInputs(), y);

    if (m_controller.HaveTrajectory()) {
        m_u = m_controller.Calculate(m_observer.Xhat());

        if (!AtGoal()) {
            m_leftGrbx.SetVoltage(units::volt_t{m_u(0)});
            m_rightGrbx.SetVoltage(units::volt_t{m_u(1)});
        } else {
            m_leftGrbx.SetVoltage(0_V);
            m_rightGrbx.SetVoltage(0_V);
        }
    } else {
        // Update previous u stored in the controller. We don't care what the
        // return value is.
        m_u = m_controller.Calculate(m_observer.Xhat());

        // Run observer predict with inputs from teleop
        m_u << m_leftGrbx.Get() * frc::RobotController::GetInputVoltage(),
            m_rightGrbx.Get() * frc::RobotController::GetInputVoltage();
    }

    Log(m_controller.GetReferences(), m_observer.Xhat(), m_u, y);

    if constexpr (frc::RobotBase::IsSimulation()) {
        auto batteryVoltage = frc::RobotController::GetInputVoltage();
        m_drivetrainSim.SetInputs(
            units::volt_t{m_leftGrbx.Get() * batteryVoltage},
            units::volt_t{m_rightGrbx.Get() * batteryVoltage});

        m_drivetrainSim.Update(GetDt());

        m_leftEncoderSim.SetDistance(
            m_drivetrainSim.GetLeftPosition().to<double>());
        m_rightEncoderSim.SetDistance(
            m_drivetrainSim.GetRightPosition().to<double>());
        m_imuSim.SetAngle(units::degree_t{
            m_drivetrainSim.GetHeading().Radians() - m_headingOffset});

        const auto& plant = DrivetrainController::GetPlant();
        Eigen::Matrix<double, 2, 1> x;
        x << m_drivetrainSim.GetLeftVelocity().to<double>(),
            m_drivetrainSim.GetRightVelocity().to<double>();
        Eigen::Matrix<double, 2, 1> u;
        u << m_leftGrbx.Get() * batteryVoltage,
            m_rightGrbx.Get() * batteryVoltage;
        Eigen::Matrix<double, 2, 1> xdot = plant.A() * x + plant.B() * u;
        m_imuSim.SetAccelInstantX(
            units::meters_per_second_squared_t{(xdot(0) + xdot(1)) / 2.0});

        m_field.SetRobotPose(m_drivetrainSim.GetPose());
    }
}

void Drivetrain::AddTrajectory(const frc::Pose2d& start,
                               const std::vector<frc::Translation2d>& interior,
                               const frc::Pose2d& end) {
    m_controller.AddTrajectory(start, interior, end);
}

void Drivetrain::AddTrajectory(const frc::Pose2d& start,
                               const std::vector<frc::Translation2d>& interior,
                               const frc::Pose2d& end,
                               const frc::TrajectoryConfig& config) {
    m_controller.AddTrajectory(start, interior, end, config);
}

void Drivetrain::AddTrajectory(const std::vector<frc::Pose2d>& waypoints) {
    m_controller.AddTrajectory(waypoints);
}

void Drivetrain::AddTrajectory(const std::vector<frc::Pose2d>& waypoints,
                               const frc::TrajectoryConfig& config) {
    m_controller.AddTrajectory(waypoints, config);
}

frc::TrajectoryConfig Drivetrain::MakeTrajectoryConfig() {
    return DrivetrainController::MakeTrajectoryConfig();
}

bool Drivetrain::AtGoal() const { return m_controller.AtGoal(); }

const Eigen::Matrix<double, 7, 1>& Drivetrain::GetStates() const {
    return m_observer.Xhat();
}

const Eigen::Matrix<double, 2, 1>& Drivetrain::GetInputs() const {
    return m_controller.GetInputs();
}

units::ampere_t Drivetrain::GetCurrentDraw() const {
    return m_drivetrainSim.GetCurrentDraw();
}

void Drivetrain::TeleopInit() {
    // If the robot was disabled while still following a trajectory in
    // autonomous, it will continue to do so in teleop. This aborts any
    // trajectories so teleop driving can occur.
    m_controller.AbortTrajectories();

    Enable();
}

void Drivetrain::RobotPeriodic() {
    using State = DrivetrainController::State;
    using Input = DrivetrainController::Input;

    const auto& xHat = m_observer.Xhat();
    m_xStateEntry.SetDouble(xHat(State::kX));
    m_yStateEntry.SetDouble(xHat(State::kY));
    m_headingStateEntry.SetDouble(xHat(State::kHeading));
    m_leftVelocityStateEntry.SetDouble(xHat(State::kLeftVelocity));
    m_rightVelocityStateEntry.SetDouble(xHat(State::kRightVelocity));
    m_leftPositionStateEntry.SetDouble(xHat(State::kLeftPosition));
    m_rightPositionStateEntry.SetDouble(xHat(State::kRightPosition));
    m_leftVoltageInputEntry.SetDouble(
        m_controller.GetInputs()(Input::kLeftVoltage));
    m_rightVoltageInputEntry.SetDouble(
        m_controller.GetInputs()(Input::kRightVoltage));
    m_headingOutputEntry.SetDouble(GetAngle().to<double>());
    m_leftPositionOutputEntry.SetDouble(GetLeftPosition().to<double>());
    m_rightPositionOutputEntry.SetDouble(GetRightPosition().to<double>());
    m_accelerationOutputEntry.SetDouble(GetAcceleration().to<double>());
}

void Drivetrain::TeleopPeriodic() {
    static frc::Joystick driveStick1{kDriveStick1Port};
    static frc::Joystick driveStick2{kDriveStick2Port};

    double y = ApplyDeadband(driveStick1.GetY(), kJoystickDeadband);
    double x = ApplyDeadband(driveStick2.GetX(), kJoystickDeadband);

    if (driveStick1.GetRawButton(1)) {
        y *= 0.5;
        x *= 0.5;
    }
    auto [left, right] = CurvatureDrive(y, x, driveStick2.GetRawButton(2));
    auto [leftLim, rightLim] = LimitAcceleration(
        units::volt_t{left * 12.0}, units::volt_t{right * 12.0}, 4.5_mps_sq);
    m_leftGrbx.SetVoltage(leftLim);
    m_rightGrbx.SetVoltage(rightLim);
}

std::tuple<units::volt_t, units::volt_t> Drivetrain::LimitAcceleration(
    units::volt_t leftVoltage, units::volt_t rightVoltage,
    units::meters_per_second_squared_t maxAccel) {
    using State = DrivetrainController::State;
    using Input = DrivetrainController::Input;

    Eigen::Matrix<double, 2, 1> x =
        m_observer.Xhat().block<2, 1>(State::kLeftVelocity, 0);
    Eigen::Matrix<double, 2, 1> u;
    u << leftVoltage.to<double>(), rightVoltage.to<double>();

    const auto plant = DrivetrainController::GetPlant();
    Eigen::Matrix<double, 2, 1> xdot = plant.A() * x + plant.B() * u;
    units::meters_per_second_squared_t a_l{xdot(0)};
    units::meters_per_second_squared_t a_r{xdot(1)};

    auto accel = (a_l + a_r) / 2.0;

    // w = (v_r - v_l) / (2r)
    // alpha = (a_r - a_l) / (2r)

    // v_l = v - w r
    // v_r = v + w r
    // a_l,new = a_new - alpha r
    // a_r,new = a_new + alpha r
    // a_l,new = a_new - alpha_old r
    // a_r,new = a_new + alpha_old r
    // a_l,new = a_new - ((a_r,old - a_l,old) / (2r)) r
    // a_r,new = a_new + ((a_r,old - a_l,old) / (2r)) r
    // a_l,new = a_new - (a_r,old - a_l,old) / 2
    // a_r,new = a_new + (a_r,old - a_l,old) / 2
    if (accel > maxAccel) {
        xdot(0) = (maxAccel - (a_r - a_l) / 2.0).to<double>();
        xdot(1) = (maxAccel + (a_r - a_l) / 2.0).to<double>();
        u = plant.B().householderQr().solve(xdot - plant.A() * x);
    } else if (accel < -maxAccel) {
        xdot(0) = (-maxAccel - (a_r - a_l) / 2.0).to<double>();
        xdot(1) = (-maxAccel + (a_r - a_l) / 2.0).to<double>();
        u = plant.B().householderQr().solve(xdot - plant.A() * x);
    }

    u = frc::NormalizeInputVector<2>(u, 12.0);

    return {units::volt_t{u(Input::kLeftVoltage)},
            units::volt_t{u(Input::kRightVoltage)}};
}
