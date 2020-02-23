// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <frc/Timer.h>
#include <rev/CANSparkMax.h>
#include <wpi/mutex.h>

#include "Constants.hpp"
#include "communications/PublishNode.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

/**
 * Provides an interface for the robot's intake. It's in front of a funnel and
 * below a conveyor that has proximity sensors on the top and bottom.
 */
class Intake : public SubsystemBase, public PublishNode {
public:
    enum class State {
        kIdle,
        kDropBalls,
        kRaiseBalls,
    };

    enum class ArmMotorDirection { kIntake, kOuttake, kIdle };

    Intake();

    /**
     * Deploys the Intake
     */
    void Deploy();

    /**
     * Stows the Intake
     */
    void Stow();

    /**
     * Returns whether or not the Intake is deployed
     */
    bool IsDeployed() const;

    /**
     * Sets the Intake motor to either Intake, Idle, or Outtake Balls based on
     * the argument passed through
     * @param state of the motor
     */
    void SetArmMotor(ArmMotorDirection armMotorState);

    /**
     * Sets the funnel motors to either intake or outtake
     * @param voltage to set motors forwards or backwards
     */
    void SetFunnel(double speed);

    /**
     * Sets all motors pertaining to the Intake subsystem to outtake their balls
     * to "feed" to our teammates
     */
    void FeedBalls();

    /**
     * Sets the Conveyor to either intake or outtake
     * @param voltage to set motors forwards or backwards
     */
    void SetConveyor(double speed);

    /**
     * Returns whether or not the upper proximity sensor detects something; true
     * means something is detected, false means it is not.
     */
    bool IsUpperSensorBlocked() const;

    /**
     * Returns whether or not the lower proximity sensor detects something; true
     * means something is detected, false means it is not.
     */
    bool IsLowerSensorBlocked() const;

    void ProcessMessage(const ButtonPacket& message) override;

    /**
     * Checks for Button Press messages
     */
    void SubsystemPeriodic() override;

private:
    State m_state = State::kIdle;

    frc::Timer m_conveyorTimer;

    rev::CANSparkMax m_funnelMotorLeft{Constants::Intake::kFunnelPortLeft,
                                       rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_funnelMotorRight{
        Constants::Intake::kFunnelPortRight,
        rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_conveyorMotor{Constants::Intake::kConveyorPort,
                                     rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_armMotor{Constants::Intake::kArmMotorPort,
                                rev::CANSparkMax::MotorType::kBrushless};
    wpi::mutex m_armMotorMutex;

    frc::DigitalInput m_upperSensor{Constants::Intake::kUpperSensorPort};
    frc::DigitalInput m_lowerSensor{Constants::Intake::kLowerSensorPort};

    frc::Solenoid m_arm{Constants::Intake::kArmPort};
};
}  // namespace frc3512
