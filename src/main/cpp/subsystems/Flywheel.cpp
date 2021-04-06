// Copyright (c) 2016-2021 FRC Team 3512. All Rights Reserved.

#include "subsystems/Flywheel.hpp"

#include <fmt/core.h>
#include <frc/DriverStation.h>
#include <frc/Joystick.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <units/math.h>
#include <wpi/math>

#include "CANSparkMaxUtil.hpp"
#include "EigenFormat.hpp"
#include "TargetModel.hpp"
#include "UnitsFormat.hpp"
#include "controllers/TurretController.hpp"
#include "subsystems/Drivetrain.hpp"

using namespace frc3512;
using namespace frc3512::Constants::Robot;

const frc::Pose2d Flywheel::kTargetPoseInGlobal{TargetModel::kCenter.X(),
                                                TargetModel::kCenter.Y(),
                                                units::radian_t{wpi::math::pi}};

Flywheel::Flywheel(Drivetrain& drivetrain)
    : ControlledSubsystemBase("Flywheel",
                              {ControllerLabel{"Angular velocity", "rad/s"}},
                              {ControllerLabel{"Voltage", "V"}},
                              {ControllerLabel{"Angular velocity", "rad/s"}}),
      m_drivetrain(drivetrain) {
    SetCANSparkMaxBusUsage(m_leftGrbx, Usage::kMinimal);
    m_leftGrbx.SetSmartCurrentLimit(40);
    SetCANSparkMaxBusUsage(m_rightGrbx, Usage::kMinimal);
    m_rightGrbx.SetSmartCurrentLimit(40);

    // Ensures CANSparkMax::Get() returns an initialized value
    m_leftGrbx.Set(0.0);
    m_rightGrbx.Set(0.0);

    m_encoder.SetDistancePerPulse(FlywheelController::kDpP);
    m_encoder.SetSamplesToAverage(5);
    m_leftGrbx.SetInverted(false);
    m_rightGrbx.SetInverted(false);

    // TODO: Update table with new values
    m_table.Insert(125_in, 450_rad_per_s);
    m_table.Insert(175_in, 463_rad_per_s);
    m_table.Insert(200_in, 472_rad_per_s);
    // The values below have been updated
    m_table.Insert(231_in, 508_rad_per_s);
    m_table.Insert(312_in, 598_rad_per_s);
    m_table.Insert(360_in, 708_rad_per_s);

    Reset();
    SetGoal(0_rad_per_s);
}

void Flywheel::SetMoveAndShoot(bool moveAndShoot) {
    m_moveAndShoot = moveAndShoot;
}

units::radian_t Flywheel::GetAngle() {
    return units::radian_t{m_encoder.GetDistance()};
}

units::radians_per_second_t Flywheel::GetAngularVelocity() const {
    return m_angularVelocity;
}

void Flywheel::SetGoal(units::radians_per_second_t velocity) {
    m_controller.SetGoal(velocity);
}

units::radians_per_second_t Flywheel::GetGoal() const {
    return m_controller.GetGoal();
}

bool Flywheel::AtGoal() const { return m_controller.AtGoal(); }

void Flywheel::SetGoalFromPose() {
    if (frc::DriverStation::GetInstance().IsTest()) {
        SetGoal(ThrottleToReference(m_testThrottle));
    } else {
        SetGoal(GetReferenceForPose(m_drivetrain.GetPose()));
    }
}

bool Flywheel::IsOn() const { return GetGoal() > 0_rad_per_s; }

bool Flywheel::IsReady() { return GetGoal() > 0_rad_per_s && AtGoal(); }

void Flywheel::Reset() {
    m_observer.Reset();
    m_controller.Reset();
    m_u = Eigen::Matrix<double, 1, 1>::Zero();
    m_encoder.Reset();
    m_angle = GetAngle();
    m_lastAngle = m_angle;
}

units::ampere_t Flywheel::GetCurrentDraw() const {
    return m_flywheelSim.GetCurrentDraw();
}

units::radians_per_second_t Flywheel::GetReferenceForPose(
    const frc::Pose2d& drivetrainPose) const {
    auto turretPose =
        TurretController::DrivetrainToTurretInGlobal(drivetrainPose);
    return m_table[turretPose.Translation().Distance(
        kTargetPoseInGlobal.Translation())];
}

void Flywheel::RobotPeriodic() {
    if (frc::DriverStation::GetInstance().IsTest()) {
        static frc::Joystick appendageStick2{kAppendageStick2Port};

        m_testThrottle = appendageStick2.GetThrottle();
        auto manualRef = ThrottleToReference(m_testThrottle);
        fmt::print("Manual angular velocity: {}\n", manualRef);
    }
}

void Flywheel::ControllerPeriodic() {
    UpdateDt();

    m_observer.Predict(m_u, GetDt());

    // Adjusts the flywheel's goal while moving and shooting
    if (IsOn() && m_moveAndShoot) {
        SetGoalFromPose();
    }

    m_angle = GetAngle();
    m_time = frc2::Timer::GetFPGATimestamp();

    // WPILib uses the time between pulses in GetRate() to calculate velocity,
    // but this is very noisy for high-resolution encoders. Instead, we
    // calculate a velocity from the change in angle over change in time, which
    // is more precise.
    m_angularVelocity = m_velocityFilter.Calculate((m_angle - m_lastAngle) /
                                                   (m_time - m_lastTime));

    Eigen::Matrix<double, 1, 1> y;
    y << GetAngularVelocity().to<double>();
    m_observer.Correct(m_controller.GetInputs(), y);
    m_u = m_controller.Calculate(m_observer.Xhat());
    SetVoltage(units::volt_t{m_u(0)});

    Log(m_controller.GetReferences(), m_observer.Xhat(), m_u, y);

    if constexpr (frc::RobotBase::IsSimulation()) {
        units::volt_t voltage{m_leftGrbx.Get() *
                              frc::RobotController::GetInputVoltage()};
        if (m_flywheelSim.GetAngularVelocity() > 0_rad_per_s) {
            voltage -= FlywheelController::kS;
        } else if (m_flywheelSim.GetAngularVelocity() < 0_rad_per_s) {
            voltage += FlywheelController::kS;
        }

        m_flywheelSim.SetInput(frc::MakeMatrix<1, 1>(voltage.to<double>()));
        m_flywheelSim.Update(GetDt());
        m_encoderSim.SetDistance(m_flywheelSim.GetAngle().to<double>());
    }

    m_lastAngle = m_angle;
    m_lastTime = m_time;
}

void Flywheel::SetSimAngularVelocity(units::radians_per_second_t velocity) {
    m_flywheelSim.SetState(frc::MakeMatrix<2, 1>(
        m_flywheelSim.GetAngle().to<double>(), velocity.to<double>()));
}

void Flywheel::SetVoltage(units::volt_t voltage) {
    m_leftGrbx.SetVoltage(voltage);
    m_rightGrbx.SetVoltage(-voltage);
}

units::radians_per_second_t Flywheel::ThrottleToReference(double throttle) {
    // 1. Remap input from [1..-1] to [0..1]
    auto remap = (1.0 - throttle) / 2.0;
    // 2. Rescale that to [400.0...800.0]
    constexpr auto kLow = 400_rad_per_s;
    constexpr auto kHigh = 800_rad_per_s;
    auto rescale = kLow + (kHigh - kLow) * remap;
    // 3. Round to the nearest radian per second
    return units::math::round(rescale);
}
