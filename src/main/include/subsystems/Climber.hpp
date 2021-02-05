// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Solenoid.h>
#include <frc/simulation/LinearSystemSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <units/length.h>
#include <units/voltage.h>

#include "Constants.hpp"
#include "NetworkTableUtil.hpp"
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
    Climber(const Climber&) = delete;
    Climber& operator=(const Climber&) = delete;

    /**
     * Returns the position of the elevator.
     */
    units::meter_t GetElevatorPosition();

    /**
     * Returns true if encoder has passed top limit.
     */
    bool HasPassedTopLimit();

    /**
     * Returns true if encoder has passed bottom limit.
     */
    bool HasPassedBottomLimit();

    /**
     * Returns the voltage applied to elevator motor.
     */
    units::volt_t GetElevatorMotorOutput() const;

    void RobotPeriodic() override;

    void TeleopPeriodic() override;

    void TestPeriodic() override;

private:
    rev::CANSparkMax m_elevator{Constants::Climber::kElevatorPortRight,
                                rev::CANSparkMax::MotorType::kBrushless};
    rev::CANEncoder m_elevatorEncoder =
        m_elevator.GetEncoder(rev::CANEncoder::EncoderType::kHallSensor);

    rev::CANSparkMax m_traverser{frc3512::Constants::Climber::kTraverserPort,
                                 rev::CANSparkMax::MotorType::kBrushless};

    frc::Solenoid m_pancake{frc3512::Constants::Climber::kClimberLock};

    Turret& m_turret;

    nt::NetworkTableEntry m_elevatorEncoderEntry =
        NetworkTableUtil::MakeEntry("/Diagnostics/Climber/Elevator encoder", 0);

    // Simulation variables
    frc::sim::LinearSystemSim<2, 1, 1> m_elevatorSim{
        frc::LinearSystemId::ElevatorSystem(frc::DCMotor::NEO(), 4.5_kg,
                                            0.86_in, 20.0)};

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
