// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/DoubleSolenoid.h>
#include <frc/Solenoid.h>
#include <rev/CANSparkMax.h>

#include "Constants.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

/**
 * The climber composes of an winch, elevator, and traverser.
 *
 * To climb, the elevator reaches for the climbing bar, then the winch grabs
 * onto the bar. The climbing bar can tilt in different directions depending on
 * how many robots are hanging on it. To keep the bar level, the transverser can
 * move left or right to equalize the robot weights on each side of the bar.
 */
class Climber : public SubsystemBase {
public:
    Climber() : SubsystemBase("Climber") {}

    /**
     *  Sets the transverser speed
     *
     *  @param speed, sets the speed of the transverser.
     */
    void SetTransverser(double speed);

    /**
     *  Sets the elevator speed
     *
     *  @param speed, sets the speed of the elevator.
     */
    void SetElevator(double speed);

    /**
     * Engages the climber solenoid to move in a forward manner
     */
    void ForwardClimb();

    /**
     * Engages the climber solenoid to move in a reverse manner
     */
    void ReverseClimb();

    /**
     * Locks or disables the climber solenoid
     */
    void LockClimb();

    void ProcessMessage(const ButtonPacket& message) override;

private:
    rev::CANSparkMax m_armLeft{frc3512::Constants::Climber::kArmPortLeft,
                               rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_armRight{frc3512::Constants::Climber::kArmPortRight,
                                rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_transverser{
        frc3512::Constants::Climber::kTransverserPort,
        rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_winch{frc3512::Constants::Climber::kWinchPort,
                             rev::CANSparkMax::MotorType::kBrushless};

    frc::Solenoid m_wranglerSolenoid{
        frc3512::Constants::Climber::kWranglerSolenoid};

    frc::DoubleSolenoid m_climbSole{
        frc3512::Constants::Climber::kClimbForwardSolenoid,
        frc3512::Constants::Climber::kClimbReverseSolenoid};
};

}  // namespace frc3512
