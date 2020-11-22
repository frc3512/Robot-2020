// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <stdint.h>

#include <frc/DutyCycleEncoder.h>
#include <frc/geometry/Pose2d.h>
#include <frc/simulation/DutyCycleEncoderSim.h>
#include <frc/simulation/LinearSystemSim.h>
#include <frc2/Timer.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <units/current.h>
#include <units/voltage.h>

#include "Constants.hpp"
#include "controllers/TurretController.hpp"
#include "rev/CANSparkMax.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

class Vision;
class Drivetrain;
class Flywheel;

/**
 * Turret subsystem.
 *
 * This is a circular plate below the shooter that can rotate to aim it.
 */
class Turret : public SubsystemBase {
public:
    enum class Direction { kNone, kCCW, kCW };

    explicit Turret(Vision& vision, Drivetrain& drivetrain, Flywheel& flywheel);

    Turret(Turret&&) = default;
    Turret& operator=(Turret&&) = default;

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
     * Returns true if the turret has reached the goal heading.
     */
    bool AtGoal() const;

    /**
     * Returns the voltage applied to turret motor.
     */
    units::volt_t GetMotorOutput() const;

    void DisabledInit() override {
        m_controller.Disable();
        SetControlMode(TurretController::ControlMode::kManual);
    }

    void AutonomousInit() override {
        m_controller.Enable();
        SetControlMode(TurretController::ControlMode::kAutoAim);
    }

    void TeleopInit() override {
        m_controller.Enable();
        SetControlMode(TurretController::ControlMode::kManual);
    }

    void RobotPeriodic() override;

    void TeleopPeriodic() override;

    void TestPeriodic() override;

    void ControllerPeriodic();

private:
    // A CCW (positive) offset makes the encoder hit the soft limit sooner when
    // rotating CCW. For the current gear ratio, the duty cycle encoder rolls
    // over to 0 rad at 0.707 rad. The offset is half that to provide a 20
    // degree buffer on each side for the robot starting configuration.
    static constexpr auto kOffset = units::radian_t{0.380};

    frc::DutyCycleEncoder m_encoder{Constants::Turret::kEncoderPort};
    rev::CANSparkMax m_motor{Constants::Turret::kPort,
                             rev::CANSparkMax::MotorType::kBrushless};

    TurretController m_controller;

    Vision& m_vision;
    Drivetrain& m_drivetrain;
    Flywheel& m_flywheel;

    uint32_t m_poseMeasurementFaultCounter = 0;
    nt::NetworkTableInstance m_inst = nt::NetworkTableInstance::GetDefault();
    nt::NetworkTableEntry m_headingEntry =
        m_inst.GetEntry("/Diagnostics/Turret/Heading");
    nt::NetworkTableEntry m_poseMeasurementFaultEntry =
        m_inst.GetEntry("/Diagnostics/Turret/Measurement fault counter");

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
