// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <stdint.h>

#include <frc/DutyCycleEncoder.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/simulation/DutyCycleEncoderSim.h>
#include <frc/simulation/LinearSystemSim.h>
#include <frc2/Timer.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <units/current.h>
#include <units/voltage.h>

#include "ADCInput.hpp"
#include "Constants.hpp"
#include "NetworkTableUtil.hpp"
#include "controllers/TurretController.hpp"
#include "rev/CANSparkMax.hpp"
#include "subsystems/ControlledSubsystemBase.hpp"

namespace frc3512 {

class Vision;
class Drivetrain;
class Flywheel;

/**
 * Turret subsystem.
 *
 * This is a circular plate below the shooter that can rotate to aim it.
 */
class Turret : public ControlledSubsystemBase<2, 1, 1> {
public:
    enum class Direction { kNone, kCCW, kCW };

    // CW limit for turret that allows climber to move
    static constexpr units::radian_t kCWLimitForClimbing{0.375 * wpi::math::pi};

    explicit Turret(Vision& vision, Drivetrain& drivetrain, Flywheel& flywheel);

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
    bool HasPassedCCWLimit() const;

    /**
     * Returns true if the encoder has passed the clockwise limit.
     */
    bool HasPassedCWLimit() const;

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
        SetControlMode(TurretController::ControlMode::kAutoAim);
    }

    void RobotPeriodic() override;

    void TeleopPeriodic() override;

    void TestPeriodic() override;

    void ControllerPeriodic() override;

private:
    // A CCW (positive) offset makes the encoder hit the soft limit sooner when
    // rotating CCW. For the current gear ratio, the duty cycle encoder rolls
    // over to 0 rad at 0.707 rad. The offset is half that to provide a 20
    // degree buffer on each side for the robot starting configuration.
    static constexpr auto kOffset = units::radian_t{0.380 + 0.062};

    units::radian_t m_ccwLimit{TurretController::kCCWLimit};
    units::radian_t m_cwLimit{TurretController::kCWLimit};

    frc::DutyCycleEncoder m_encoder{Constants::Turret::kEncoderPort};
    rev::CANSparkMax m_motor{Constants::Turret::kPort,
                             rev::CANSparkMax::MotorType::kBrushless};

    frc::LinearSystem<2, 1, 1> m_plant{TurretController::GetPlant()};
    frc::KalmanFilter<2, 1, 1> m_observer{
        m_plant, {0.21745, 0.28726}, {0.01}, Constants::kDt};
    TurretController m_controller;

    ADCInput m_leftLimitSwitch{Constants::Turret::kLeftHallPort};
    ADCInput m_rightLimitSwitch{Constants::Turret::kRightHallPort};

    Vision& m_vision;
    Drivetrain& m_drivetrain;
    Flywheel& m_flywheel;

    uint32_t m_poseMeasurementFaultCounter = 0;
    nt::NetworkTableEntry m_angleStateEntry =
        NetworkTableUtil::MakeEntry("/Diagnostics/Turret/States/Angle", 0);
    nt::NetworkTableEntry m_angularVelocityStateEntry =
        NetworkTableUtil::MakeEntry(
            "/Diagnostics/Turret/States/Angular velocity", 0);
    nt::NetworkTableEntry m_inputVoltageEntry =
        NetworkTableUtil::MakeEntry("/Diagnostics/Turret/Inputs/Voltage", 0);
    nt::NetworkTableEntry m_angleOutputEntry =
        NetworkTableUtil::MakeEntry("/Diagnostics/Turret/Outputs/Angle", "0");
    nt::NetworkTableEntry m_controlModeEntry =
        NetworkTableUtil::MakeEntry("/Diagnostics/Turret/Control mode", false);
    nt::NetworkTableEntry m_poseMeasurementFaultEntry =
        NetworkTableUtil::MakeEntry(
            "/Diagnostics/Turret/Measurement fault counter", 0);

    // Simulation variables
    frc::sim::LinearSystemSim<2, 1, 1> m_turretSim{m_controller.GetPlant(),
                                                   {0.001}};
    frc::sim::DutyCycleEncoderSim m_encoderSim{m_encoder};

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
