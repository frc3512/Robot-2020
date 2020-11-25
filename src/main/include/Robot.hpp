// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <string>
#include <vector>

#include <frc/TimedRobot.h>
#include <frc/logging/CSVLogFile.h>
#include <frc/simulation/JoystickSim.h>
#include <frc2/Timer.h>

#if RUNNING_FRC_TESTS
#include <gtest/gtest.h>
#else
#define EXPECT_EQ(a, b)
#define EXPECT_FALSE(a)
#define EXPECT_GT(a, b)
#define EXPECT_LT(a, b)
#define EXPECT_TRUE(a)
#endif

#include "AutonomousChooser.hpp"
#include "Constants.hpp"
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

    /**
     * Start shooting.
     */
    void Shoot();

    /**
     * Returns true if currently shooting.
     */
    bool IsShooting() const;

    void SimulationInit() override;

    void DisabledInit() override;

    void AutonomousInit() override;

    void TeleopInit() override;

    void TestInit() override;

    void RobotPeriodic() override;

    void SimulationPeriodic() override;

    void DisabledPeriodic() override;

    void AutonomousPeriodic() override;

    void TeleopPeriodic() override;

    void TestPeriodic() override;

    /**
     * Runs all the controller update functions.
     */
    void ControllerPeriodic();

    /**
     * Runs the shooter state machine.
     */
    void RunShooterSM();

    /**
     * Initialization code for no-op autonomous.
     */
    void AutoNoOpInit();

    /**
     * Initialization code for driving towards the allied alliance station from
     * the initiation line and in front of the loading zone.
     */
    void AutoLoadingZoneDriveForwardInit();

    /**
     * Initialization code for shooting three power cells from the initiation
     * line in front of the loading zone.
     */
    void AutoLoadingZoneShootThreeInit();

    /**
     * Initialization code for shooting three power cells from the initiation
     * line in front of the target zone.
     */
    void AutoTargetZoneShootThreeInit();

    /**
     * Initialization code for driving towards the allied alliance station from
     * the initiation line in front of the allied trench run.
     */
    void AutoRightSideDriveForwardInit();

    /**
     * Initialization code for shooting three power cells from the initiation
     * line in front of the allied trench run.
     */
    void AutoRightSideShootThreeInit();

    /**
     * Initialization code for shooting three power cells from the initiation
     * line in front of the opposing trench run and intaking two balls in the
     * trench run.
     */
    void AutoLeftSideIntakeInit();

    /**
     * Initialization code for shooting three power cells from the initiation
     * line in front of the allied trench run and intaking three balls in the
     * trench run.
     */
    void AutoRightSideIntakeInit();

    /**
     * Initialization code for shooting three power cells from the initiation
     * line in front of the allied trench run, intaking three balls in the
     * trench run, then shooting them.
     */
    void AutoRightSideShootSixInit();

    /**
     * Initialization code for shooting three power cells from the initiation
     * line in front of the target zone, intaking two balls under the power
     * generator, then shooting them.
     */
    void AutoTargetZoneShootSixInit();

    /**
     * Periodic code for no-op autonomous.
     */
    void AutoNoOpPeriodic();

    /**
     * Periodic code for driving towards the allied alliance station from the
     * initiation line and in front of the loading zone.
     */
    void AutoLoadingZoneDriveForwardPeriodic();

    /**
     * Periodic code for shooting three power cells from the initiation line in
     * front of the loading zone.
     */
    void AutoLoadingZoneShootThreePeriodic();

    /**
     * Periodic code for shooting three power cells from the initiation line in
     * front of the target zone.
     */
    void AutoTargetZoneShootThreePeriodic();

    /**
     * Periodic code for driving towards the allied alliance station from the
     * initiation line in front of the allied trench run.
     */
    void AutoRightSideDriveForwardPeriodic();

    /**
     * Periodic code for shooting three power cells from the initiation line in
     * front of the allied trench run.
     */
    void AutoRightSideShootThreePeriodic();

    /**
     * Periodic code for shooting three power cells from the initiation line in
     * front of the opposing trench run and intaking two balls in the trench
     * run.
     */
    void AutoLeftSideIntakePeriodic();

    /**
     * Periodic code for shooting three power cells from the initiation line in
     * front of the allied trench run and intaking three balls in the trench
     * run.
     */
    void AutoRightSideIntakePeriodic();

    /**
     * Periodic code for shooting three power cells from the initiation line in
     * front of the allied trench run, intaking three balls in the trench run,
     * then shooting them.
     */
    void AutoRightSideShootSixPeriodic();

    /**
     * Periodic code for shooting three power cells from the initiation line in
     * front of the target zone, intaking two balls under the power generator,
     * then shooting them.
     */
    void AutoTargetZoneShootSixPeriodic();

    /**
     * Sets the selected autonomous mode for testing purposes.
     *
     * @param name The autonomous mode's name passed to
     *             AutonomousChooser::AddAutonomous().
     */
    void SelectAutonomous(wpi::StringRef name);

    /**
     * Returns the names of autonomous modes to test.
     */
    const std::vector<std::string>& GetAutonomousNames() const;

    /**
     * Assertions to check at the end of each autonomous mode during unit
     * testing.
     */
    void ExpectAutonomousEndConds();

private:
    // The order the subsystems are initialized determines the order the
    // controllers run in.
    Vision m_vision;
    Drivetrain m_drivetrain;
    Flywheel m_flywheel{m_drivetrain};
    Turret m_turret{m_vision, m_drivetrain, m_flywheel};
    Intake m_intake{m_flywheel};
    Climber m_climber;

    ShootingState m_state = ShootingState::kIdle;
    frc2::Timer m_timer;

    AutonomousChooser m_autonChooser{"No-op", [=] { AutoNoOpInit(); },
                                     [=] { AutoNoOpPeriodic(); }};

    frc::CSVLogFile m_batteryLogger{"Battery", "Battery voltage (V)"};

    frc::sim::JoystickSim m_driveStick1{Constants::Robot::kDriveStick1Port};
    frc::sim::JoystickSim m_driveStick2{Constants::Robot::kDriveStick2Port};
    frc::sim::JoystickSim m_appendageStick1{
        Constants::Robot::kAppendageStick1Port};
    frc::sim::JoystickSim m_appendageStick2{
        Constants::Robot::kAppendageStick2Port};
};

}  // namespace frc3512
