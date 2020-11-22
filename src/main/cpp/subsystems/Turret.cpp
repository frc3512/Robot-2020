// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Turret.hpp"

#include <frc/Joystick.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>

#include "CANSparkMaxUtil.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Flywheel.hpp"
#include "subsystems/Vision.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Robot;

Turret::Turret(Vision& vision, Drivetrain& drivetrain, Flywheel& flywheel)
    : m_vision(vision), m_drivetrain(drivetrain), m_flywheel(flywheel) {
    SetCANSparkMaxBusUsage(m_motor, Usage::kMinimal);

    // Ensures CANSparkMax::Get() returns an initialized value
    m_motor.Set(0.0);

    m_encoder.SetDistancePerRotation(TurretController::kDpR);
    Reset(0_rad);
}

void Turret::SetControlMode(TurretController::ControlMode mode) {
    m_controller.SetControlMode(mode);
}

void Turret::SetDirection(Direction direction) {
    if (m_controller.GetControlMode() ==
        TurretController::ControlMode::kManual) {
        if (direction == Direction::kCW) {
            SetVoltage(-7_V);
        } else if (direction == Direction::kCCW) {
            SetVoltage(7_V);
        } else {
            SetVoltage(0_V);
        }
    }
}

void Turret::Reset(units::radian_t initialHeading) {
    m_controller.Reset(initialHeading);

    if constexpr (frc::RobotBase::IsSimulation()) {
        Eigen::Matrix<double, 2, 1> xHat;
        xHat << initialHeading.to<double>(), 0.0;
        m_turretSim.SetState(xHat);

        m_encoderSim.SetDistance(HeadingToEncoderDistance(initialHeading));
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
        SetControlMode(TurretController::ControlMode::kManual);
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

void Turret::TestPeriodic() {
    static frc::Joystick appendageStick1{kAppendageStick1Port};

    // Ignore soft limits so the user can manually reset the turret before
    // rebooting the robot
    int pov = appendageStick1.GetPOV();
    if (pov == 90) {
        // CW
        m_motor.SetVoltage(-7_V);
    } else if (pov == 270) {
        // CCW
        m_motor.SetVoltage(7_V);
    } else {
        m_motor.SetVoltage(0_V);
    }
}

void Turret::ControllerPeriodic() {
    m_controller.SetDrivetrainStates(m_drivetrain.GetStates());
    m_controller.SetFlywheelReferences(
        m_flywheel.GetReferenceForPose(m_drivetrain.GetPose()));

    Eigen::Matrix<double, 1, 1> y;
    y << GetAngle().to<double>();
    Eigen::Matrix<double, 1, 1> u = m_controller.UpdateAndLog(y);

    auto globalMeasurement = m_vision.GetGlobalMeasurement();
    // TODO: Reenable when vision measurements are reliable
    if (0) {
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

        // If pose measurement is too far away from the state estimate, discard
        // it and increment the fault counter
        if (m_drivetrain.GetPose().Translation().Distance(
                drivetrainInGlobal.Translation()) < 1_m) {
            m_drivetrain.CorrectWithGlobalOutputs(
                drivetrainInGlobal.X(), drivetrainInGlobal.Y(),
                globalMeasurement.value().timestamp);
        } else {
            m_poseMeasurementFaultCounter++;
            m_poseMeasurementFaultEntry.SetDouble(
                m_poseMeasurementFaultCounter);
        }
    }

    // Set motor input
    if (m_controller.GetControlMode() !=
        TurretController::ControlMode::kManual) {
        SetVoltage(units::volt_t{u(0)});
    }

    if constexpr (frc::RobotBase::IsSimulation()) {
        m_turretSim.SetInput(frc::MakeMatrix<1, 1>(
            m_motor.Get() * frc::RobotController::GetInputVoltage()));

        m_turretSim.Update(m_controller.GetDt());

        m_encoderSim.SetDistance(HeadingToEncoderDistance(
            units::radian_t{m_turretSim.GetOutput(0)}));
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

double Turret::HeadingToEncoderDistance(units::radian_t heading) {
    // heading = -GetDistance() + kOffset
    // -GetDistance() = heading - kOffset
    // GetDistance() = kOffset - heading
    return (kOffset - heading).to<double>();
}
