// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Drivetrain.hpp"

#include <frc/Joystick.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/simulation/SimDeviceSim.h>

#include "CANSparkMaxUtil.hpp"
#include "controllers/DrivetrainController.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Robot;

Drivetrain::Drivetrain()
    : m_controller{new DrivetrainController},
      m_drivetrainSim{m_controller->GetPlant(), DrivetrainController::kWidth,
                      frc::DCMotor::NEO(2),
                      DrivetrainController::kDriveGearRatio,
                      DrivetrainController::kWheelRadius} {
    frc::sim::SimDeviceSim gyroSim{"ADXRS450_Gyro[0]"};
    m_angleSim = gyroSim.GetDouble("Angle");

    SetCANSparkMaxBusUsage(m_leftMaster, Usage::kMinimal);
    SetCANSparkMaxBusUsage(m_leftSlave, Usage::kMinimal);
    SetCANSparkMaxBusUsage(m_rightMaster, Usage::kMinimal);
    SetCANSparkMaxBusUsage(m_rightSlave, Usage::kMinimal);

    m_drive.SetDeadband(kJoystickDeadband);

    // Ensures CANSparkMax::Get() returns an initialized value
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

void Drivetrain::Drive(double xSpeed, double zRotation, bool allowTurnInPlace) {
    m_drive.CurvatureDrive(xSpeed, zRotation, allowTurnInPlace);
}

frc::Pose2d Drivetrain::GetPose() const {
    const auto& x = m_controller->GetStates();
    return frc::Pose2d{
        units::meter_t{x(DrivetrainController::State::kX)},
        units::meter_t{x(DrivetrainController::State::kY)},
        units::radian_t{x(DrivetrainController::State::kHeading)}};
}

units::radian_t Drivetrain::GetAngle() const {
    return units::degree_t{-m_gyro.GetAngle()} + m_headingOffset;
}

units::radians_per_second_t Drivetrain::GetAngularRate() const {
    return units::degrees_per_second_t{-m_gyro.GetRate()};
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
    m_controller->Reset(initialPose, initialPose);
    m_leftEncoder.Reset();
    m_rightEncoder.Reset();
    m_gyro.Reset();
    m_headingOffset = initialPose.Rotation().Radians();

    if constexpr (frc::RobotBase::IsSimulation()) {
        Eigen::Matrix<double, 7, 1> x = Eigen::Matrix<double, 7, 1>::Zero();

        using State = frc::sim::DifferentialDrivetrainSim::State;
        x(State::kX) = initialPose.X().to<double>();
        x(State::kY) = initialPose.Y().to<double>();
        x(State::kHeading) = initialPose.Rotation().Radians().to<double>();
        m_drivetrainSim.SetState(x);
    }
}

void Drivetrain::EnableController() {
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
    Eigen::Matrix<double, 3, 1> y;
    y << GetAngle().to<double>(), GetLeftPosition().to<double>(),
        GetRightPosition().to<double>();
    Eigen::Matrix<double, 2, 1> u = m_controller->UpdateAndLog(y);

    if (!m_controller->IsOpenLoop()) {
        // Set motor inputs
        m_leftGrbx.SetVoltage(units::volt_t{u(0)});
        m_rightGrbx.SetVoltage(units::volt_t{u(1)});
    }

    if constexpr (frc::RobotBase::IsSimulation()) {
        auto batteryVoltage = frc::RobotController::GetInputVoltage();
        m_drivetrainSim.SetInputs(
            units::volt_t{m_leftGrbx.Get() * batteryVoltage},
            units::volt_t{m_rightGrbx.Get() * batteryVoltage});

        m_drivetrainSim.Update(Constants::kDt);

        using State = frc::sim::DifferentialDrivetrainSim::State;
        m_leftEncoderSim.SetDistance(
            m_drivetrainSim.GetState(State::kLeftPosition));
        m_rightEncoderSim.SetDistance(
            m_drivetrainSim.GetState(State::kRightPosition));

        m_angleSim.Set(-units::degree_t{
            units::radian_t{m_drivetrainSim.GetState(State::kHeading)} -
            m_headingOffset}
                            .to<double>());

        frc::sim::RoboRioSim::SetVInVoltage(
            frc::sim::BatterySim::Calculate({m_drivetrainSim.GetCurrentDraw()})
                .to<double>());
    }
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

void Drivetrain::RobotPeriodic() {
    m_leftEncoderEntry.SetDouble(GetLeftPosition().to<double>());
    m_rightEncoderEntry.SetDouble(GetRightPosition().to<double>());
    m_headingEntry.SetDouble(GetAngle().to<double>());
}

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
