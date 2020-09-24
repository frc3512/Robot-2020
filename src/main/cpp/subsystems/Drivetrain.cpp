// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Drivetrain.hpp"

#include <frc/Joystick.h>
#include <frc/RobotController.h>

#include "CANSparkMaxUtil.hpp"
#include "controllers/DrivetrainController.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Robot;

Drivetrain::Drivetrain() : m_controller{new DrivetrainController} {
    SetCANSparkMaxBusUsage(m_leftMaster, Usage::kMinimal);
    SetCANSparkMaxBusUsage(m_leftSlave, Usage::kMinimal);
    SetCANSparkMaxBusUsage(m_rightMaster, Usage::kMinimal);
    SetCANSparkMaxBusUsage(m_rightSlave, Usage::kMinimal);

    m_drive.SetDeadband(kJoystickDeadband);

    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);

    m_leftGrbx.SetInverted(true);
    m_drive.SetRightSideInverted(false);

    m_leftEncoder.SetSamplesToAverage(10);
    m_rightEncoder.SetSamplesToAverage(10);

    m_leftEncoder.SetReverseDirection(false);
    m_rightEncoder.SetReverseDirection(true);
    m_leftEncoder.SetDistancePerPulse(DrivetrainController::kDpP);
    m_rightEncoder.SetDistancePerPulse(DrivetrainController::kDpP);
}

Drivetrain::~Drivetrain() {}

void Drivetrain::Drive(double throttle, double turn, bool isQuickTurn) {
    std::lock_guard lock(m_motorControllerMutex);
    m_drive.CurvatureDrive(throttle, turn, isQuickTurn);
}

units::radian_t Drivetrain::GetAngle() const {
    return units::degree_t{-m_gyro.GetAngle()} + m_headingOffset;
}

units::radians_per_second_t Drivetrain::GetAngularRate() const {
    return units::degrees_per_second_t{-m_gyro.GetRate()};
}

void Drivetrain::ResetGyro() { m_gyro.Reset(); }

void Drivetrain::CalibrateGyro() { m_gyro.Calibrate(); }

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

void Drivetrain::ResetEncoders() {
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
}

void Drivetrain::Reset(const frc::Pose2d& initialPose) {
    m_controller->Reset(initialPose, initialPose);
    ResetEncoders();
    ResetGyro();
    m_headingOffset = initialPose.Rotation().Radians();
}

void Drivetrain::EnableController() {
    m_lastTime = std::chrono::steady_clock::now();
    m_controller->Enable();
    m_drive.SetSafetyEnabled(false);
}

void Drivetrain::DisableController() {
    m_controller->Disable();
    m_drive.SetSafetyEnabled(true);
}

bool Drivetrain::IsControllerEnabled() const {
    return m_controller->IsEnabled();
}

void Drivetrain::CorrectWithGlobalOutputs(units::meter_t x, units::meter_t y,
                                          int64_t timestamp) {
    m_controller->CorrectWithGlobalOutputs(x, y, timestamp);
}

void Drivetrain::ControllerPeriodic() {
    m_controller->SetMeasuredInputs(
        units::volt_t{m_leftGrbx.Get() *
                      frc::RobotController::GetInputVoltage()},
        units::volt_t{m_rightGrbx.Get() *
                      frc::RobotController::GetInputVoltage()});
    m_controller->SetMeasuredLocalOutputs(GetAngle(), GetLeftPosition(),
                                          GetRightPosition());

    auto now = std::chrono::steady_clock::now();
    m_controller->Update(now - m_lastTime, now - GetStartTime());

    if (!m_controller->IsOpenLoop()) {
        // Set motor inputs
        auto u = m_controller->GetInputs();
        m_leftGrbx.SetVoltage(units::volt_t{u(0)});
        m_rightGrbx.SetVoltage(units::volt_t{u(1)});
    }
    m_lastTime = now;
}

void Drivetrain::SetWaypoints(const frc::Pose2d& start,
                              const std::vector<frc::Translation2d>& interior,
                              const frc::Pose2d& end) {
    m_controller->SetWaypoints(start, interior, end);
}

void Drivetrain::SetWaypoints(const frc::Pose2d& start,
                              const std::vector<frc::Translation2d>& interior,
                              const frc::Pose2d& end,
                              frc::TrajectoryConfig& config) {
    m_controller->SetWaypoints(start, interior, end, config);
}

bool Drivetrain::AtGoal() const { return m_controller->AtGoal(); }

Eigen::Matrix<double, 10, 1> Drivetrain::GetNextXhat() const {
    return m_controller->GetStates();
}

void Drivetrain::DisabledInit() { DisableController(); }

void Drivetrain::AutonomousInit() {
    EnableController();
    m_controller->SetOpenLoop(false);
}

void Drivetrain::TeleopInit() { m_controller->SetOpenLoop(true); }

void Drivetrain::TeleopPeriodic() {
    static frc::Joystick driveStick1{kDriveStick1Port};
    static frc::Joystick driveStick2{kDriveStick2Port};

    double y = driveStick1.GetY();
    double x = driveStick2.GetX();

    if (driveStick1.GetRawButton(1)) {
        y *= 0.5;
        x *= 0.5;
    }
    Drive(y, x, driveStick2.GetRawButton(2));
}
