// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/I2C.h>
#include <frc/Servo.h>
#include <frc/Solenoid.h>
#include <frc/simulation/LinearSystemSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/util/Color.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/ColorSensorV3.h>
#include <units/length.h>
#include <units/voltage.h>

#include "HWConfig.hpp"
#include "NetworkTableUtil.hpp"
#include "rev/CANEncoder.hpp"
#include "rev/CANSparkMax.hpp"
#include "rev/ColorMatch.hpp"
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
    enum class ControlPanelState { kInit, kRotateWheel, kStopOnColor };

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
    rev::CANSparkMax m_elevator{HWConfig::Climber::kElevatorPortRight,
                                rev::CANSparkMax::MotorType::kBrushless};
    rev::CANEncoder m_elevatorEncoder =
        m_elevator.GetEncoder(rev::CANEncoder::EncoderType::kHallSensor);

    rev::CANSparkMax m_traverser{frc3512::HWConfig::Climber::kTraverserPort,
                                 rev::CANSparkMax::MotorType::kBrushless};

    frc::Solenoid m_pancake{frc3512::HWConfig::Climber::kClimberLock};

    Turret& m_turret;

    // Control panel variables
    static constexpr frc::Color kRedTarget{0.561, 0.232, 0.114};
    static constexpr frc::Color kGreenTarget{0.197, 0.561, 0.240};
    static constexpr frc::Color kBlueTarget{0.143, 0.427, 0.429};
    static constexpr frc::Color kYellowTarget{0.361, 0.524, 0.113};

    ControlPanelState m_state = ControlPanelState::kInit;
    frc::Servo m_colorSensorArm{HWConfig::Climber::kColorSensorArmPort};
    rev::ColorSensorV3 m_colorSensor{frc::I2C::Port::kMXP};
    rev::ColorMatch m_matcher;
    frc::Color m_currentColor;
    frc::Color m_startColor;
    frc::Color m_prevColor;
    double m_confidence;
    int m_changedColorCount;

    // Networktable entries
    nt::NetworkTableEntry m_elevatorEncoderEntry =
        NetworkTableUtil::MakeDoubleEntry(
            "/Diagnostics/Climber/Elevator encoder");

    nt::NetworkTableEntry m_colorSensorOutputEntry =
        NetworkTableUtil::MakeStringEntry(
            "/Diagnostics/Climber/Color Sensor Output");

    nt::NetworkTableEntry m_colorStateMachineEntry =
        NetworkTableUtil::MakeStringEntry(
            "/Diagnostics/Climber/Color State Machine Entry");

    nt::NetworkTableEntry m_changedColorNumEntry =
        NetworkTableUtil::MakeDoubleEntry(
            "/Diagnostics/Climber/Changed Color Count");

    // Simulation variables
    frc::sim::LinearSystemSim<2, 1, 1> m_elevatorSim{
        frc::LinearSystemId::ElevatorSystem(frc::DCMotor::NEO(), 4.5_kg,
                                            0.86_in, 20.0)};

    /**
     * Sets the traverser speed.
     *
     * The traverser is also used to spin the control panel.
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

    /**
     * Runs the control panel state machine.
     *
     * This state machine either rotates the control panel 3.5 rotations or
     * stops the control panel on a certain color based on button press. The
     * control panel will be stopped so the field color sensor reads the color
     * desired by the Field Management System (FMS).
     *
     * A 90° offset is applied to the FMS color because the robot's color sensor
     * isn't directly under the same color as the field color sensor.
     */
    void RunControlPanelSM();
};

}  // namespace frc3512
