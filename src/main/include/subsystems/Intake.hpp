// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/DigitalInput.h>
#include <frc/Solenoid.h>
#include <frc/Timer.h>
#include <frc/logging/CSVLogFile.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <rev/CANSparkMax.h>
#include <units/time.h>

#include "HWConfig.hpp"
#include "NetworkTableUtil.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

class Flywheel;

/**
 * The intake subsystem.
 *
 * The intake consists of intake rollers, a funnel behind that, and a conveyor
 * above that. The conveyor has proximity sensors on the top and bottom.
 */
class Intake : public SubsystemBase {
public:
    /**
     * Arm motor direction.
     */
    enum class ArmMotorDirection { kIntake, kOuttake, kIdle };

    /**
     * Constructs an Intake.
     *
     * @param flywheel Flywheel subsystem.
     */
    explicit Intake(Flywheel& flywheel);

    Intake(const Intake&) = delete;
    Intake& operator=(const Intake&) = delete;

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
     * Sets funnel and arm motors to intake balls.
     */
    void Start();

    /**
     * Turns off the funnel and arm motors.
     */
    void Stop();

    /**
     * Sets all motors pertaining to the Intake subsystem to outtake their balls
     * to "feed" to our teammates
     */
    void FeedBalls();

    /**
     * Sets the Conveyor to either intake or outtake
     * @param speed voltage to set motors forwards or backwards
     */
    void SetConveyor(double speed);

    /**
     * Returns true if the conveyor is running.
     */
    bool IsConveyorRunning() const;

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

    void RobotPeriodic() override;

private:
    frc::Timer m_conveyorTimer;

    Flywheel& m_flywheel;

    rev::CANSparkMax m_funnelMotorLeft{HWConfig::Intake::kFunnelLeftMotorID,
                                       rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_funnelMotorRight{
        HWConfig::Intake::kFunnelRightMotorID,
        rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_conveyorMotor{HWConfig::Intake::kConveyorMotorID,
                                     rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_armMotor{HWConfig::Intake::kArmMotorID,
                                rev::CANSparkMax::MotorType::kBrushless};

    frc::DigitalInput m_upperSensor{HWConfig::Intake::kUpperSensorChannel};
    frc::DigitalInput m_lowerSensor{HWConfig::Intake::kLowerSensorChannel};

    frc::Solenoid m_arm{frc::PneumaticsModuleType::CTREPCM,
                        HWConfig::Intake::kArmChannel};

    nt::NetworkTableEntry m_upperSensorEntry = NetworkTableUtil::MakeBoolEntry(
        "/Diagnostics/Intake/Upper sensor blocked");
    nt::NetworkTableEntry m_lowerSensorEntry = NetworkTableUtil::MakeBoolEntry(
        "/Diagnostics/Intake/Lower sensor blocked");

    frc::CSVLogFile m_intakeLog{"Intake", "Deployed (bool)", "Speed (-1 .. 1)"};

    /**
     * Sets the intake motor to either Intake, Idle, or Outtake balls.
     *
     * @param direction The intake direction.
     */
    void SetArmMotor(ArmMotorDirection direction);

    /**
     * Sets the funnel motors's speed.
     *
     * @param speed Funnel motor speed (-1 to 1).
     */
    void SetFunnel(double speed);
};

}  // namespace frc3512
