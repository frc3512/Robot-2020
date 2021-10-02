// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <stdint.h>

#include <frc/DigitalInput.h>
#include <frc/DutyCycle.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/simulation/AnalogInputSim.h>
#include <frc/simulation/DutyCycleEncoderSim.h>
#include <frc/simulation/LinearSystemSim.h>
#include <networktables/NetworkTableEntry.h>
#include <units/angle.h>
#include <units/current.h>
#include <units/voltage.h>

#include "ADCInput.hpp"
#include "Constants.hpp"
#include "HWConfig.hpp"
#include "NetworkTableUtil.hpp"
#include "controllers/TurretController.hpp"
#include "rev/CANSparkMax.hpp"
#include "subsystems/ControlledSubsystemBase.hpp"
#include "subsystems/Vision.hpp"

namespace frc3512 {

class Drivetrain;
class Flywheel;

/**
 * The turret subsystem.
 *
 * This is a circular plate below the shooter that can rotate to aim it.
 *
 * The turret uses a Kalman filter for state estimation.
 */
class Turret : public ControlledSubsystemBase<2, 1, 1> {
public:
    /**
     * Turret motor direction.
     */
    enum class Direction { kNone, kCCW, kCW };

    /// CW limit for turret that allows climber to move.
    static constexpr units::radian_t kCWLimitForClimbing{0.375 *
                                                         wpi::numbers::pi};

    /**
     * Producer-consumer queue for yaw measurement from Vision
     * subsystem.
     */
    static_concurrent_queue<Vision::GlobalMeasurement, 8> visionQueue;

    /**
     * Constructs a Turret.
     *
     * @param drivetrain Drivetrain subsystem.
     * @param flywheel Flywheel subsystem.
     */
    explicit Turret(Drivetrain& drivetrain, Flywheel& flywheel);

    Turret(const Turret&) = delete;
    Turret& operator=(const Turret&) = delete;

    /**
     * Sets turret control mode.
     *
     * @param mode Control mode.
     */
    void SetControlMode(TurretController::ControlMode mode);

    /**
     * Set direction turret should move during manual override.
     */
    void SetDirection(Direction direction);

    /**
     * Resets the controller.
     *
     * @param initialHeading The initial turret heading in the drivetrain frame.
     */
    void Reset(units::radian_t initialHeading);

    /**
     * Returns the angle of the turret.
     *
     * @return angle in radians
     */
    units::radian_t GetAngle() const;

    /**
     * Returns true if the encoder has passed the counterclockwise limit.
     */
    bool HasPassedCCWLimit();

    /**
     * Returns true if the encoder has passed the clockwise limit.
     */
    bool HasPassedCWLimit();

    /**
     * Set the limit for the CCW side of the turret
     */
    void SetCCWLimit(units::radian_t limit);

    /**
     * Set the limit for CW side of the turret
     */
    void SetCWLimit(units::radian_t limit);

    /**
     * Sets the end goal of the controller profile.
     *
     * @param angleGoal           Desired heading.
     * @param angularVelocityGoal Desired heading rate.
     */
    void SetGoal(units::radian_t angleGoal,
                 units::radians_per_second_t angularVelocityGoal);

    /**
     * Returns true if the turret has reached the goal heading.
     */
    bool AtGoal() const;

    /**
     * Returns the voltage applied to turret motor.
     */
    units::volt_t GetMotorOutput() const;

    /**
     * Returns transformation from turret in global frame to drivetrain in
     * global frame.
     */
    frc::Transform2d GetTurretInGlobalToDrivetrainInGlobal();

    /**
     * Returns the turret state estimate.
     */
    const Eigen::Matrix<double, 2, 1>& GetStates() const;

    void DisabledInit() override {
        Disable();
        SetControlMode(TurretController::ControlMode::kManual);
    }

    void AutonomousInit() override {
        Enable();
        SetControlMode(TurretController::ControlMode::kAutoAim);
    }

    void TeleopInit() override {
        Enable();
        SetControlMode(TurretController::ControlMode::kClosedLoop);
    }

    void TeleopPeriodic() override;

    void TestPeriodic() override;

    void ControllerPeriodic() override;

private:
    // A CCW (positive) offset makes the encoder hit the soft limit sooner when
    // rotating CCW. For the current gear ratio, the duty cycle encoder rolls
    // over to 0 rad at 5.655 rad. The offset is half that to provide a 162
    // degree buffer on each side for the robot starting configuration.
    static constexpr auto kOffset = units::radian_t{2.6138};

    units::radian_t m_ccwLimit{TurretController::kCCWLimit};
    units::radian_t m_cwLimit{TurretController::kCWLimit};

    frc::DigitalInput m_encoderInput{HWConfig::Turret::kEncoderChannel};
    frc::DutyCycle m_dutyCycle{m_encoderInput};
    frc::DutyCycleEncoder m_encoder{m_dutyCycle};
    rev::CANSparkMax m_motor{HWConfig::Turret::kMotorID,
                             rev::CANSparkMax::MotorType::kBrushless};
    nt::NetworkTableEntry m_encoderEntry = NetworkTableUtil::MakeDoubleEntry(
        "Diagnostics/Turret/Duty Cycle Output");

    nt::NetworkTableEntry m_controllerYawEntry =
        NetworkTableUtil::MakeDoubleEntry("Diagnostics/Turret/Controller Yaw");

    frc::LinearSystem<2, 1, 1> m_plant{TurretController::GetPlant()};
    frc::KalmanFilter<2, 1, 1> m_observer{
        m_plant, {0.21745, 0.28726}, {0.01}, Constants::kControllerPeriod};

    TurretController m_controller;
    Eigen::Matrix<double, 1, 1> m_u = Eigen::Matrix<double, 1, 1>::Zero();

    ADCInput m_ccwLimitSwitch{HWConfig::Turret::kCCWHallChannel};
    ADCInput m_cwLimitSwitch{HWConfig::Turret::kCWHallChannel};

    Drivetrain& m_drivetrain;
    Flywheel& m_flywheel;

    frc::Transform2d cameraInGlobalToDrivetrainInGlobal;

    // Simulation variables
    frc::sim::LinearSystemSim<2, 1, 1> m_turretSim{m_controller.GetPlant(),
                                                   {0.001}};
    frc::sim::DutyCycleEncoderSim m_encoderSim{m_encoder};
    frc::sim::AnalogInputSim m_ccwLimitSwitchSim{
        HWConfig::Turret::kCCWHallChannel};
    frc::sim::AnalogInputSim m_cwLimitSwitchSim{
        HWConfig::Turret::kCWHallChannel};

    /**
     * Set voltage output of turret motor.
     *
     * This also applies soft limits using the turret encoder.
     */
    void SetVoltage(units::volt_t voltage);

    /**
     * Converts given turret heading to DutyCycleEncoder GetDistance() result.
     *
     * @param heading The heading to convert.
     */
    static double HeadingToEncoderDistance(units::radian_t heading);
};

}  // namespace frc3512
