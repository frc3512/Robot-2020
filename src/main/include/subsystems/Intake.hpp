// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/DigitalInput.h>
#include <frc/DoubleSolenoid.h>
#include <frc/logging/CSVLogFile.h>
#include <frc2/Timer.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <units/time.h>

#include "Constants.hpp"
#include "rev/CANSparkMax.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

class Flywheel;

/**
 * Intake subsystem.
 *
 * The intake consists of intake rollers, a funnel behind that, and a conveyor
 * above that. The conveyor has proximity sensors on the top and bottom.
 */
class Intake : public SubsystemBase {
public:
    enum class ArmMotorDirection { kIntake, kOuttake, kIdle };

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
     * Sets the intake motor to either Intake, Idle, or Outtake balls.
     *
     * @param direction The intake direction.
     */
    void SetArmMotor(ArmMotorDirection direction);

    /**
     * Sets the funnel motors to either intake or outtake
     * @param speed voltage to set motors forwards or backwards
     */
    void SetFunnel(double speed);

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

    void TeleopPeriodic() override;

private:
    frc2::Timer m_conveyorTimer;

    Flywheel& m_flywheel;

    rev::CANSparkMax m_funnelMotorLeft{Constants::Intake::kFunnelPortLeft,
                                       rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_funnelMotorRight{
        Constants::Intake::kFunnelPortRight,
        rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_conveyorMotor{Constants::Intake::kConveyorPort,
                                     rev::CANSparkMax::MotorType::kBrushless};

    rev::CANSparkMax m_armMotor{Constants::Intake::kArmMotorPort,
                                rev::CANSparkMax::MotorType::kBrushless};

    frc::DigitalInput m_upperSensor{Constants::Intake::kUpperSensorPort};
    frc::DigitalInput m_lowerSensor{Constants::Intake::kLowerSensorPort};

    frc::DoubleSolenoid m_arm{Constants::Intake::kArmForward,
                              Constants::Intake::kArmReverse};

    nt::NetworkTableInstance m_inst = nt::NetworkTableInstance::GetDefault();
    nt::NetworkTableEntry m_upperSensorEntry =
        m_inst.GetEntry("/Diagnostics/Intake/Upper sensor blocked");
    nt::NetworkTableEntry m_lowerSensorEntry =
        m_inst.GetEntry("/Diagnostics/Intake/Lower sensor blocked");

    frc::CSVLogFile m_intakeLog{"Intake", "Deployed (bool)", "Speed (-1 .. 1)"};
    units::second_t m_startTime = frc2::Timer::GetFPGATimestamp();
};

}  // namespace frc3512
