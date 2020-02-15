// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/DoubleSolenoid.h>
#include <frc/Joystick.h>
#include <frc/PowerDistributionPanel.h>
#include <frc/Solenoid.h>
#include <rev/CANSparkMax.h>

#include "Constants.hpp"
#include "communications/PublishNode.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

class Climber : public SubsystemBase, public PublishNode {
public:
    Climber();

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

    void ProcessMessage(const ButtonPacket& message);

private:
    rev::CANSparkMax m_armLeft{frc3512::Constants::Climber::kArmPortLeft,
                               rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_armRight{kArmPortRight,
                                rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_transverser{
        frc3512::Constants::Climber::kTransverserPort,
        rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_winch{frc3512::Constants::Climber::kWinchPort,
                             rev::CANSparkMax::MotorType::kBrushless};

    frc::Solenoid m_wranglerSolenoid{
        frc3512::Constants::Climber::kWranglerSolenoid};
    
    frc::DoubleSolenoid m_climbSole{frc3512::Constants::Climber::kClimbForwardSolenoid, frc3512::Constants::Climber::kClimbReverseSolenoid};

};

}  // namespace frc3512
