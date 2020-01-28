// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <limits>

#include <frc/DigitalInput.h>
#include <frc/Encoder.h>
#include <rev/CANSparkMax.h>

#include "Constants.hpp"
#include "communications/PublishNode.hpp"
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
class Turret : public SubsystemBase, public PublishNode {
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
     *  Resets the encoder wired to the Turret.
     */
    void ResetEncoder();

    bool GetLeftHallTriggered() const;

    bool GetRightHallTriggered() const;

private:
    // Encoder
    frc::Encoder m_encoder{Constants::Turret::kEncoderA,
                           Constants::Turret::kEncoderB};

    // Both hall sensors for each boundary of the turret
    frc::DigitalInput m_rightHall{Constants::Turret::kRightHallPort};
    frc::DigitalInput m_leftHall{Constants::Turret::kLeftHallPort};

    // Spark Max
    rev::CANSparkMax m_motor{Constants::Turret::kPort,
                             rev::CANSparkMax::MotorType::kBrushless};
};
}  // namespace frc3512
