// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/Turret.hpp"

#include <frc/DriverStation.h>
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
    : ControlledSubsystemBase("Turret",
                              {ControllerLabel{"Angle", "rad"},
                               ControllerLabel{"Angular velocity", "rad/s"}},
                              {ControllerLabel{"Voltage", "V"}},
                              {ControllerLabel{"Angle", "rad"}}),
      m_vision(vision),
      m_drivetrain(drivetrain),
      m_flywheel(flywheel) {
    SetCANSparkMaxBusUsage(m_motor, Usage::kMinimal);
    m_motor.SetSmartCurrentLimit(80);

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
    Eigen::Matrix<double, 2, 1> xHat;
    xHat << initialHeading.to<double>(), 0.0;

    m_observer.Reset();
    m_observer.SetXhat(xHat);
    m_controller.Reset(initialHeading);
    m_u = Eigen::Matrix<double, 1, 1>::Zero();

    if constexpr (frc::RobotBase::IsSimulation()) {
        m_turretSim.SetState(xHat);
        m_encoderSim.SetDistance(HeadingToEncoderDistance(initialHeading));

        if (units::radian_t{m_turretSim.GetOutput(0)} >
            TurretController::kCCWLimit) {
            m_ccwLimitSwitchSim.SetVoltage(0.0);
        } else {
            m_ccwLimitSwitchSim.SetVoltage(5.0);
        }
        if (units::radian_t{m_turretSim.GetOutput(0)} <
            TurretController::kCWLimit) {
            m_cwLimitSwitchSim.SetVoltage(0.0);
        } else {
            m_cwLimitSwitchSim.SetVoltage(5.0);
        }
    }
}

units::radian_t Turret::GetAngle() const {
    // Negative sign makes encoder counterclockwise positive. An offset is added
    // to make the absolute encoder return 0 rad when it's facing forward.
    return units::radian_t{-m_encoder.GetDistance()} + kOffset;
}

bool Turret::HasPassedCCWLimit() {
    return GetAngle() > m_ccwLimit || !m_ccwLimitSwitch.Get();
}

bool Turret::HasPassedCWLimit() {
    return GetAngle() < m_cwLimit || !m_cwLimitSwitch.Get();
}

void Turret::SetCCWLimit(units::radian_t limit) { m_ccwLimit = limit; }

void Turret::SetCWLimit(units::radian_t limit) { m_cwLimit = limit; }

void Turret::SetGoal(units::radian_t angleGoal,
                     units::radians_per_second_t angularVelocityGoal) {
    m_controller.SetGoal(angleGoal, angularVelocityGoal);
}

bool Turret::AtGoal() const { return m_controller.AtGoal(); }

units::volt_t Turret::GetMotorOutput() const {
    return m_motor.Get() *
           units::volt_t{frc::RobotController::GetInputVoltage()};
}

const Eigen::Matrix<double, 2, 1>& Turret::GetStates() const {
    return m_observer.Xhat();
}

void Turret::RobotPeriodic() {
    m_angleStateEntry.SetDouble(
        m_observer.Xhat()(TurretController::State::kAngle));
    m_angularVelocityStateEntry.SetDouble(
        m_observer.Xhat()(TurretController::State::kAngularVelocity));
    m_inputVoltageEntry.SetDouble(m_controller.GetInputs()(0));
    m_angleOutputEntry.SetDouble(GetAngle().to<double>());
    m_ccwLimitSwitchValueEntry.SetBoolean(m_ccwLimitSwitch.Get());
    m_cwLimitSwitchValueEntry.SetBoolean(m_cwLimitSwitch.Get());

    int controlMode = static_cast<int>(m_controller.GetControlMode());
    if (controlMode == 0) {
        m_controlModeEntry.SetString("Manual");
    } else if (controlMode == 1) {
        m_controlModeEntry.SetString("ClosedLoop");
    } else if (controlMode == 2) {
        m_controlModeEntry.SetString("AutoAim");
    }
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
    UpdateDt();

    m_observer.Predict(m_u, GetDt());

    m_controller.SetDrivetrainStates(m_drivetrain.GetStates());
    m_controller.SetFlywheelReferences(
        m_flywheel.GetReferenceForPose(m_drivetrain.GetPose()));

    Eigen::Matrix<double, 1, 1> y;
    y << GetAngle().to<double>();
    m_observer.Correct(m_controller.GetInputs(), y);

    auto globalMeasurement = m_vision.GetGlobalMeasurement();
    if (globalMeasurement.has_value()) {
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

    m_u = m_controller.Calculate(m_observer.Xhat());

    // Set motor input
    if (m_controller.GetControlMode() !=
        TurretController::ControlMode::kManual) {
        SetVoltage(units::volt_t{m_u(0)});
    }

    Log(m_controller.GetReferences(), m_observer.Xhat(), m_u, y);

    if constexpr (frc::RobotBase::IsSimulation()) {
        m_turretSim.SetInput(frc::MakeMatrix<1, 1>(
            m_motor.Get() * frc::RobotController::GetInputVoltage()));

        m_turretSim.Update(GetDt());

        m_encoderSim.SetDistance(HeadingToEncoderDistance(
            units::radian_t{m_turretSim.GetOutput(0)}));

        if (units::radian_t{m_turretSim.GetOutput(0)} >
            TurretController::kCCWLimit) {
            m_ccwLimitSwitchSim.SetVoltage(0.0);
        } else {
            m_ccwLimitSwitchSim.SetVoltage(5.0);
        }
        if (units::radian_t{m_turretSim.GetOutput(0)} <
            TurretController::kCWLimit) {
            m_cwLimitSwitchSim.SetVoltage(0.0);
        } else {
            m_cwLimitSwitchSim.SetVoltage(5.0);
        }
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
