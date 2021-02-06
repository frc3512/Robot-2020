// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/Drivetrain.hpp"

#include <frc/Joystick.h>
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/StateSpaceUtil.h>
#include <frc/smartdashboard/SmartDashboard.h>
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
           ControllerLabel{"Right position", "m"}}) {
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
    return units::degree_t{-m_gyro.GetAngle()} + m_headingOffset;
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

void Drivetrain::Reset(const frc::Pose2d& initialPose) {
    using State = frc::sim::DifferentialDrivetrainSim::State;

    m_observer.Reset();
    m_controller.Reset(initialPose);
    m_u = Eigen::Matrix<double, 2, 1>::Zero();
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
    m_gyro.Reset();
    m_headingOffset = initialPose.Rotation().Radians();

    Eigen::Matrix<double, 7, 1> xHat;
    xHat(State::kX) = initialPose.X().to<double>();
    xHat(State::kY) = initialPose.Y().to<double>();
    xHat(State::kHeading) = initialPose.Rotation().Radians().to<double>();
    xHat.block<4, 1>(3, 0).setZero();
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

    Eigen::Matrix<double, 3, 1> y;
    y << frc::AngleModulus(GetAngle()).to<double>(),
        GetLeftPosition().to<double>(), GetRightPosition().to<double>();
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
        m_gyroSim.SetAngle(-units::degree_t{
            m_drivetrainSim.GetHeading().Radians() - m_headingOffset});
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
    m_leftGrbx.SetVoltage(left * 12_V);
    m_rightGrbx.SetVoltage(right * 12_V);
}
