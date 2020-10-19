// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Turret.hpp"

#include <frc/Joystick.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <frc/simulation/SimDeviceSim.h>

#include "CANSparkMaxUtil.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Vision.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Robot;

Turret::Turret(Vision& vision, Drivetrain& drivetrain)
    : m_vision(vision), m_drivetrain(drivetrain) {
    frc::sim::SimDeviceSim encoderSim{"DutyCycleEncoder[0]"};
    m_positionSim = encoderSim.GetDouble("Position");

    SetCANSparkMaxBusUsage(m_motor, Usage::kMinimal);

    // Ensures CANSparkMax::Get() returns an initialized value
    m_motor.Set(0.0);

    m_encoder.SetDistancePerRotation(TurretController::kDpR);
    Reset();
}

void Turret::SetDirection(Direction direction) {
    if (m_manualOverride) {
        if (direction == Direction::kCW) {
            SetVoltage(-7_V);
        } else if (direction == Direction::kCCW) {
            SetVoltage(7_V);
        } else {
            SetVoltage(0_V);
        }
    }
}

void Turret::SetManualOverride(bool enable) { m_manualOverride = enable; }

void Turret::Reset(units::radian_t initialHeading) {
    m_controller.Reset();

    if constexpr (frc::RobotBase::IsSimulation()) {
        Eigen::Matrix<double, 2, 1> x = Eigen::Matrix<double, 2, 1>::Zero();
        m_turretSim.SetState(x);

        m_positionSim.Set(HeadingToEncoderTurns(initialHeading));
    }
}

units::radian_t Turret::GetAngle() const {
    // Negative sign makes encoder counterclockwise positive. An offset is added
    // to make the absolute encoder return 0 rad when it's facing forward.
    return units::radian_t{-m_encoder.GetDistance()} + kOffset;
}

bool Turret::HasPassedCCWLimit() const {
    return GetAngle() > TurretController::kCCWLimit;
}

bool Turret::HasPassedCWLimit() const {
    return GetAngle() < TurretController::kCWLimit;
}

frc::Pose2d Turret::GetNextPose() const { return m_controller.GetNextPose(); }

void Turret::EnableController() { m_controller.Enable(); }

void Turret::DisableController() { m_controller.Disable(); }

bool Turret::AtGoal() const { return m_controller.AtGoal(); }

units::volt_t Turret::GetMotorOutput() const {
    return m_motor.Get() *
           units::volt_t{frc::RobotController::GetInputVoltage()};
}

void Turret::RobotPeriodic() {
    m_headingEntry.SetDouble(GetAngle().to<double>());
}

void Turret::TeleopPeriodic() {
    static frc::Joystick appendageStick1{kAppendageStick1Port};

    // Turret manual override
    if (appendageStick1.GetRawButtonPressed(11)) {
        SetManualOverride(true);
    }

    // Turrret manual spin
    int pov = appendageStick1.GetPOV();
    if (pov == 90) {
        SetDirection(Direction::kCW);
    } else if (pov == 270) {
        SetDirection(Direction::kCCW);
    } else {
        SetDirection(Direction::kNone);
    }
}

void Turret::ControllerPeriodic() {
    m_controller.SetDrivetrainStates(m_drivetrain.GetStates());

    Eigen::Matrix<double, 1, 1> y;
    y << GetAngle().to<double>();
    Eigen::Matrix<double, 1, 1> u = m_controller.UpdateAndLog(y);

    auto globalMeasurement = m_vision.GetGlobalMeasurement();
    if (globalMeasurement) {
        auto turretInGlobal = globalMeasurement.value();

        frc::Transform2d turretInGlobalToDrivetrainInGlobal{
            frc::Pose2d{
                TurretController::kDrivetrainToTurretFrame.Translation(),
                TurretController::kDrivetrainToTurretFrame.Rotation()
                        .Radians() +
                    GetAngle()},
            frc::Pose2d{}};
        auto drivetrainInGlobal =
            turretInGlobal.pose.TransformBy(turretInGlobalToDrivetrainInGlobal);

        m_drivetrain.CorrectWithGlobalOutputs(
            drivetrainInGlobal.X(), drivetrainInGlobal.Y(),
            globalMeasurement.value().timestamp);
    }

    // Set motor input
    if (!m_manualOverride) {
        SetVoltage(units::volt_t{u(0)});
    }

    if constexpr (frc::RobotBase::IsSimulation()) {
        m_turretSim.SetInput(frc::MakeMatrix<1, 1>(
            m_motor.Get() * frc::RobotController::GetInputVoltage()));

        m_turretSim.Update(Constants::kDt);

        m_positionSim.Set(
            HeadingToEncoderTurns(units::radian_t{m_turretSim.GetOutput(0)}));
    }
}

void Turret::SetVoltage(units::volt_t voltage) {
    if (voltage < 0_V && !HasPassedCWLimit()) {
        m_motor.SetVoltage(voltage);
    } else if (voltage > 0_V && !HasPassedCCWLimit()) {
        m_motor.SetVoltage(voltage);
    } else {
        m_motor.SetVoltage(0_V);
    }
}

double Turret::HeadingToEncoderTurns(units::radian_t heading) {
    // heading = -GetDistance() + kOffset
    // heading = -(Get() * kDpR) + kOffset
    // heading - kOffset = -Get() * kDpR
    // (heading - kOffset) / kDpR = -Get()
    // Get() = (kOffset - heading) / kDpR
    return (kOffset - heading).to<double>() / TurretController::kDpR;
}
