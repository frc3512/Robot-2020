// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <limits>

#include <frc/DigitalInput.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Encoder.h>
#include <frc/RTNotifier.h>
#include <frc/Timer.h>
#include <rev/CANSparkMax.h>

#include "Constants.hpp"
#include "controllers/TurretController.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

/**
 *  State class which stores numerical values as "states" for the Turret to use.
 */
enum TurretState { kIDLE = 0, kMostLeft, KMostRight };

/**
 * Subsystem specifically designed for the Turret (bottom, movable part of the
 * Shooter)
 */
class Turret : public SubsystemBase {
public:
    Turret();
    Turret(const Turret&) = delete;
    Turret& operator=(const Turret&) = delete;

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

    void Reset();

    /**
     * Returns the output of the left hall effect
     *
     * @return 'true' means triggered
     */
    bool GetLeftHallTriggered() const;

    /**
     * Returns the output of the right hall effect
     *
     * @return 'true' means triggered
     */
    bool GetRightHallTriggered() const;

    units::radian_t GetAngle();

    units::radians_per_second_t GetAngularVelocity();

    /**
     * Enables the controller.
     */
    void EnableController();

    /**
     * Disables the controller.
     */
    void DisableController();

    /**
     * Updates the controller from sensors and the motors from the controller.
     */
    void Iterate();

    void DisabledInit() override { DisableController(); }

    void AutonomousInit() override { EnableController(); }

    void TeleopInit() override { EnableController(); }

private:
#ifndef RUNNING_FRC_TESTS
    frc::DutyCycleEncoder m_encoder{Constants::Turret::kEncoderPort};
#else
    frc::Encoder m_encoder{0, 1};
#endif

    frc::DigitalInput m_rightHall{Constants::Turret::kRightHallPort};
    frc::DigitalInput m_leftHall{Constants::Turret::kLeftHallPort};

    rev::CANSparkMax m_motor{Constants::Turret::kPort,
                             rev::CANSparkMax::MotorType::kBrushless};

    frc::RTNotifier m_thread{Constants::kControllerPrio, &Turret::Iterate,
                             this};
    std::chrono::steady_clock::time_point m_lastTime =
        std::chrono::steady_clock::now();

    units::radian_t m_lastAngle;
    std::chrono::steady_clock::time_point m_timeSinceLastAngle =
        std::chrono::steady_clock::now();

    std::chrono::steady_clock::time_point m_startTime =
        std::chrono::steady_clock::now();
    TurretController m_controller;
};
}  // namespace frc3512
