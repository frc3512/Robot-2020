// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Solenoid.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <units/length.h>

#include "Constants.hpp"
#include "rev/CANEncoder.hpp"
#include "rev/CANSparkMax.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

class Turret;

/**
 * The climber composes of an elevator and a traverser.
 *
 * To climb, the elevator reaches for the climbing bar, which can tilt in
 * different directions depending on how many robots are hanging on it. To keep
 * the bar level, the traverser can move left or right to equalize the robot
 * weights on each side of the bar.
 */
class Climber : public SubsystemBase {
public:
    explicit Climber(Turret& turret);
    Climber(Climber&&) = default;
    Climber& operator=(Climber&&) = default;

    /**
     * Returns the position of the elevator.
     */
    units::meter_t GetElevatorPosition();

    void RobotPeriodic() override;

    void TeleopPeriodic() override;

    void TestPeriodic() override;

private:
    rev::CANSparkMax m_elevator{Constants::Climber::kElevatorPortRight,
                                rev::CANSparkMax::MotorType::kBrushless};
    rev::CANEncoder m_elevatorEncoder{
        m_elevator, rev::CANEncoder::EncoderType::kHallSensor};

    rev::CANSparkMax m_traverser{frc3512::Constants::Climber::kTraverserPort,
                                 rev::CANSparkMax::MotorType::kBrushless};

    frc::Solenoid m_pancake{frc3512::Constants::Climber::kClimberLock};

    Turret& m_turret;

    nt::NetworkTableInstance m_inst = nt::NetworkTableInstance::GetDefault();
    nt::NetworkTableEntry m_elevatorEncoderEntry =
        m_inst.GetEntry("/Diagnostics/Climber/Elevator encoder");

    /**
     * Sets the traverser speed.
     *
     * @param speed The speed of the traverser [-1..1].
     */
    void SetTraverser(double speed);

    /**
     * Sets the elevator speed.
     *
     * @param speed The speed of the elevator [-1..1].
     */
    void SetElevator(double speed);
};

}  // namespace frc3512
