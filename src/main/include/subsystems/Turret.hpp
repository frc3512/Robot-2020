// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <atomic>

#include <frc/DutyCycleEncoder.h>
#include <frc/geometry/Pose2d.h>
#include <frc2/Timer.h>
#include <rev/CANSparkMax.h>
#include <units/angle.h>
#include <units/voltage.h>

#include "Constants.hpp"
#include "controllers/TurretController.hpp"
#include "subsystems/ControllerSubsystemBase.hpp"

namespace frc3512 {

class Vision;
class Drivetrain;

/**
 * Subsystem specifically designed for the Turret (bottom, movable part of the
 * Shooter)
 */
class Turret : public ControllerSubsystemBase {
public:
    enum class Direction { kNone, kCCW, kCW };

    explicit Turret(Vision& vision, Drivetrain& drivetrain);

    Turret(Turret&&) = default;
    Turret& operator=(Turret&&) = default;

    /**
     * If called, permanently enables manual override.
     */
    void SetManualOverride();

    /**
     * Set direction turret should move during manual override.
     */
    void SetDirection(Direction direction);

    /**
     *  Set the velocity of the Spark Max, which is wired to the Turret.
     *
     *  Can be seen as a way of having it turn in a certain direction.
     *  This depends on the value given to it.
     *
     *  @param velocity between [-1.0 ... 1.0]
     */
    void SetVoltage(units::volt_t voltage);

    /**
     *  Resets encoder
     */
    void ResetEncoder();

    /**
     * Resets the controller
     */
    void Reset();

    /**
     * Returns the angle of the turret
     *
     * @return angle in radians
     */
    units::radian_t GetAngle() const;

    /**
     * Returns if encoder has passed the counter-clockwise limit
     *
     * @return 'true' means triggered
     */
    bool HasPassedCCWLimit() const;

    /**
     * Returns if encoder has passed the clockwise limit
     *
     * @return 'true' means triggered
     */
    bool HasPassedCWLimit() const;

    /**
     * Returns the projected pose of the turret.
     */
    frc::Pose2d GetNextPose() const;

    /**
     * Enables the controller.
     */
    void EnableController();

    /**
     * Disables the controller.
     */
    void DisableController();

    void ControllerPeriodic() override;

    void DisabledInit() override { DisableController(); }

    void AutonomousInit() override { EnableController(); }

    void TeleopInit() override { EnableController(); }

    void TeleopPeriodic() override;

private:
    // A CCW (positive) offset makes the encoder hit the soft limit sooner when
    // rotating CCW
    static constexpr units::radian_t kOffset = 30_deg;

    frc::DutyCycleEncoder m_encoder{Constants::Turret::kEncoderPort};
    rev::CANSparkMax m_motor{Constants::Turret::kPort,
                             rev::CANSparkMax::MotorType::kBrushless};

    units::second_t m_lastTime = frc2::Timer::GetFPGATimestamp();

    TurretController m_controller;

    Vision& m_vision;
    Drivetrain& m_drivetrain;

    // TODO: Let the turret move on its own once the turret encoder is trusted
    // more
    std::atomic<bool> m_manualOverride{true};
};
}  // namespace frc3512
