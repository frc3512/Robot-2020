// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <rev/CANSparkMax.h>

#include "Constants.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

/**
 * The climber composes of an elevator and a traverser.
 *
 * To climb, the elevator reaches for the climbing bar, which can tilt in
 * different directions depending on how many robots are hanging on it. To keep
 * the bar level, the transverser can move left or right to equalize the robot
 * weights on each side of the bar.
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

    void ProcessMessage(const HIDPacket& message) override;

private:
    rev::CANSparkMax m_elevator{Constants::Climber::kElevatorPortRight,
                                rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_transverser{
        frc3512::Constants::Climber::kTransverserPort,
        rev::CANSparkMax::MotorType::kBrushless};
};

}  // namespace frc3512
