// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/logging/CSVLogFile.h>
#include <frc/simulation/JoystickSim.h>
#include <frc2/Timer.h>

#include "Constants.hpp"
#include "autonselector/AutonSelector.hpp"
#include "subsystems/Climber.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Flywheel.hpp"
#include "subsystems/Intake.hpp"
#include "subsystems/Turret.hpp"
#include "subsystems/Vision.hpp"

namespace frc3512 {

using namespace frc3512::Constants::Robot;

/**
 * The main robot class.
 */
class Robot : public frc::TimedRobot {
public:
    /**
     * States used for the multi-subsystem shooting procedure
     */
    enum class ShootingState { kIdle, kStartFlywheel, kStartConveyor };

    Robot();

    void SimulationInit() override;

    void DisabledInit() override;

    void AutonomousInit() override;

    void TeleopInit() override;

    void RobotPeriodic() override;

    void SimulationPeriodic() override;

    void DisabledPeriodic() override;

    void AutonomousPeriodic() override;

    void TeleopPeriodic() override;

    /**
     * Initialization code for driving towards the allied alliance station from
     * a position on the initializating line aligned with the loading zone
     * during autonomous.
     */
    void AutoLoadingZoneDriveForwardInit();

    /**
     * Initialization code for shooting three power cells from a position on the
     * initializating line aligned with the loading zone during autonomous.
     */
    void AutoLoadingZoneShootThreeInit();

    /**
     * Initialization code for shooting three power cells from a position on the
     * initializating line aligned with the target zone during autonomous.
     */
    void AutoTargetZoneShootThreeInit();

    /**
     * Initialization code for driving towards the allied alliance station from
     * a position on the initializating line aligned with the allied trench run
     * during autonomous.
     */
    void AutoRightSideDriveForwardInit();

    /**
     * Initialization code for shooting three power cells from a position on the
     * initializating line aligned with the allied trench run during autonomous.
     */
    void AutoRightSideShootThreeInit();

    /**
     * Periodic code for driving towards the allied alliance station from a
     * position on the initializating line aligned with the loading zone during
     * autonomous.
     */
    void AutoLoadingZoneDriveForwardPeriodic();

    /**
     * Periodic code for shooting three power cells from a position on the
     * initializating line aligned with the loading zone during autonomous.
     */
    void AutoLoadingZoneShootThreePeriodic();

    /**
     * Periodic code for shooting three power cells from a position on the
     * initializating line aligned with the target zone during autonomous.
     */
    void AutoTargetZoneShootThreePeriodic();

    /**
     * Periodic code for driving towards the allied alliance station from a
     * position on the initializating line aligned with the allied trench run
     * during autonomous.
     */
    void AutoRightSideDriveForwardPeriodic();

    /**
     * Periodic code for shooting three power cells from a position on the
     * initializating line aligned with the allied trench run during autonomous.
     */
    void AutoRightSideShootThreePeriodic();

private:
    static constexpr units::meter_t kPathWeaverFudge{-0.343};

    // The order the subsystems are initialized determines the order the
    // controllers run in.
    Vision m_vision;
    Drivetrain m_drivetrain;
    Turret m_turret{m_vision, m_drivetrain};
    Flywheel m_flywheel{m_turret};
    Intake m_intake{m_flywheel};
    Climber m_climber;

    ShootingState m_state = ShootingState::kIdle;
    frc2::Timer m_timer;

    AutonSelector m_autonSelector{kDsPort};

    frc::CSVLogFile m_batteryLogger{"Battery", "Battery voltage (V)"};

    frc::sim::JoystickSim m_driveStick1{Constants::Robot::kDriveStick1Port};
    frc::sim::JoystickSim m_driveStick2{Constants::Robot::kDriveStick2Port};
    frc::sim::JoystickSim m_appendageStick1{
        Constants::Robot::kAppendageStick1Port};
    frc::sim::JoystickSim m_appendageStick2{
        Constants::Robot::kAppendageStick2Port};
};

}  // namespace frc3512
