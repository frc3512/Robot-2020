// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/logging/CSVLogFile.h>
#include <frc2/Timer.h>
#include <networktables/NetworkTableEntry.h>

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

class Robot : public frc::TimedRobot {
public:
    /**
     * States used for the multi-subsystem shooting procedure
     */
    enum class ShootingState { kIdle, kStartFlywheel, kStartConveyor };

    Robot();

    void DisabledInit() override;

    void AutonomousInit() override;

    void TeleopInit() override;

    void RobotPeriodic() override;

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
    const units::meter_t kPathWeaverFudge = -0.343_m;
    // The order the subsystems are initialized determines the order the
    // controllers run in.
    Vision m_vision;
    Drivetrain m_drivetrain{m_vision};
    Turret m_turret{m_vision, m_drivetrain};
    Flywheel m_flywheel{m_turret};
    Intake m_intake{m_flywheel};
    Climber m_climber;

    ShootingState m_state = ShootingState::kIdle;
    frc2::Timer m_timer;

    nt::NetworkTableEntry m_flywheelEntry;
    nt::NetworkTableEntry m_drivetrainLeftEntry;
    nt::NetworkTableEntry m_drivetrainRightEntry;
    nt::NetworkTableEntry m_drivetrainGyroEntry;
    nt::NetworkTableEntry m_turretEntry;
    nt::NetworkTableEntry m_upperConveyorEntry;
    nt::NetworkTableEntry m_lowerConveyorEntry;

    AutonSelector m_autonSelector{kDsPort};

    frc::CSVLogFile m_batteryLogger{"Battery", "Battery voltage (V)"};
};
}  // namespace frc3512
