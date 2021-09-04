// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <string>
#include <vector>

#include <frc/TimesliceRobot.h>
#include <frc/logging/CSVLogFile.h>
#include <frc2/Timer.h>
#include <units/time.h>

#if RUNNING_FRC_TESTS
#include <gtest/gtest.h>
#else
namespace frc3512::testing {
/**
 * A stream object shim for the GoogleTest EXPECT macros.
 */
struct NoOp {
    /**
     * No-op.
     */
    void operator<<(const char*) {}
};
}  // namespace frc3512::testing
#define EXPECT_EQ(a, b) frc3512::testing::NoOp()
#define EXPECT_FALSE(a) frc3512::testing::NoOp()
#define EXPECT_GT(a, b) frc3512::testing::NoOp()
#define EXPECT_LT(a, b) frc3512::testing::NoOp()
#define EXPECT_NEAR(a, b, c) frc3512::testing::NoOp()
#define EXPECT_TRUE(a) frc3512::testing::NoOp()
#endif

#include "AutonomousChooser.hpp"
#include "IntakeSim.hpp"
#include "NetworkTableUtil.hpp"
#include "subsystems/Climber.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Flywheel.hpp"
#include "subsystems/Intake.hpp"
#include "subsystems/Turret.hpp"
#include "subsystems/Vision.hpp"

namespace frc3512 {

/**
 * The main robot class.
 */
class Robot : public frc::TimesliceRobot {
public:
    /**
     * States used for the multi-subsystem shooting procedure
     */
    enum class ShootingState { kIdle, kStartFlywheel, kStartConveyor };

    /// Maximum time for which to run flywheel.
    static constexpr auto kMaxShootTimeout = 3_s;

    // The order the subsystems are initialized determines the in which order
    // the controllers are run.

    /// Drivetrain subsystem.
    Drivetrain drivetrain;

    /// Flywheel subsystem.
    Flywheel flywheel{drivetrain};

    /// Intake subsystem.
    Intake intake{flywheel};

    /// Turret subsystem.
    Turret turret{drivetrain, flywheel};

    /// Climber subsystem.
    Climber climber{turret};

    /// Vision subsystem.
    Vision vision{turret};

    /// Intake simulation.
    IntakeSim intakeSim;

    Robot();

    ~Robot();

    /**
     * Start shooting.
     *
     * If the user provides a number of balls to shoot, the shooter will run
     * until it detects that many dips and recoveries after initially reaching
     * the angular velocity goal. After that point, the flywheel and intake will
     * turn off. If no number is given, a timeout will be used to stop the
     * flywheel instead.
     *
     * @param ballsToShoot Number of balls to shoot.
     */
    void Shoot(int ballsToShoot = -1);

    /**
     * Start shooting.
     *
     * If the user provides a number of balls to shoot, the shooter will run
     * until it detects that many dips and recoveries after initially reaching
     * the angular velocity goal. After that point, the flywheel and intake will
     * turn off. If no number is given, a timeout will be used to stop the
     * flywheel instead. This time the number of radians the flywheel will speed
     * up to will also be set by the user instead of being picked by the lookup
     * table.
     *
     * @param radsToShoot radians the flywheel will shoot at.
     * @param ballsToShoot Number of balls to shoot.
     */
    void Shoot(units::radians_per_second_t radsToShoot, int ballsToShoot = -1);

    /**
     * Returns true if currently shooting.
     */
    bool IsShooting() const;

    /**
     * Returns true if flywheel is at its goal.
     */
    bool FlywheelAtGoal() const;

    /**
     * Returns the selected autonomous mode's expected duration.
     */
    units::second_t SelectedAutonomousDuration() const;

    /**
     * Robot-wide simulation initialization code should go here.
     *
     * Users should override this method for default Robot-wide simulation
     * related initialization which will be called when the robot is first
     * started. It will be called exactly one time after RobotInit is called
     * only when the robot is in simulation.
     */
    void SimulationInit() override;

    /**
     * Initialization code for disabled mode should go here.
     */
    void DisabledInit() override;

    /**
     * Initialization code for autonomous mode should go here.
     */
    void AutonomousInit() override;

    /**
     * Initialization code for teleop mode should go here.
     */
    void TeleopInit() override;

    /**
     * Initialization coe for test mode should go here.
     */
    void TestInit() override;

    /**
     * Periodic code for all modes should go here.
     */
    void RobotPeriodic() override;

    /**
     * Periodic simulation code should go here.
     *
     * This function is called in a simulated robot after user code executes.
     */
    void SimulationPeriodic() override;

    /**
     * Periodic code for disabled mode should go here.
     */
    void DisabledPeriodic() override;

    /**
     * Periodic code for autonomous mode should go here.
     */
    void AutonomousPeriodic() override;

    /**
     * Periodic code for teleop mode should go here.
     */
    void TeleopPeriodic() override;

    /**
     * Periodic code for test mode should go here.
     */
    void TestPeriodic() override;

    /**
     * Runs the shooter state machine.
     */
    void RunShooterSM();

    /**
     * No-op autonomous.
     */
    void AutoNoOp();

    /**
     * Shoot three power cells from the initiation line in front of the opposing
     * trench run and intake two balls in the trench run.
     */
    void AutoLeftSideIntake();

    /**
     * 10-ball autonomous.
     */
    void AutoLeftSideShootTen();

    /**
     * Drive towards the allied alliance station from the initiation line and in
     * front of the loading zone.
     */
    void AutoLoadingZoneDriveForward();

    /**
     * Shoot three power cells from the initiation line in front of the loading
     * zone.
     */
    void AutoLoadingZoneShootThree();

    /**
     * Shoot three power cells from the initiation line in front of the target
     * zone.
     */
    void AutoTargetZoneShootThree();

    /**
     * Shoot three power cells from the initiation line in front of the target
     * zone, intake three balls in the trench run, then shoot them.
     */
    void AutoTargetZoneShootSix();

    /**
     * Drive towards the allied alliance station from the initiation line in
     * front of the allied trench run.
     */
    void AutoRightSideDriveForward();

    /**
     * Shoot three power cells from the initiation line in front of the allied
     * trench run and intake three balls in the trench run.
     */
    void AutoRightSideIntake();

    /**
     * Shoot three power cells from the initiation line in front of the allied
     * trench run.
     */
    void AutoRightSideShootThree();

    /**
     * Shoot three power cells from the initiation line in front of the allied
     * trench run, intake three balls in the trench run, then shoot them.
     */
    void AutoRightSideShootSix();

    /**
     * Follows either the red or blue path for Path A and B defined in the
     * At-Home Challenges manual depending on input given by the ultrasonic
     * sensors for completing the Galactic Search Challenge.
     */
    void AutoGalacticSearch();

    /**
     * Shoot three power cells from the initiation line in front of the allied
     * trench run, intake three balls in the trench run, intake two balls under
     * shield generator, then shoot all five.
     */
    void AutoRightSideShootEight();

    /**
     * Follow the "bounce" path defined in the 2021 At-Home Challenges manual.
     */
    void AutoNavBounce();

    /**
     * Follow the "barrel racing" path defined in the 2021 At-Home Challenges
     * manual.
     */
    void AutoNavBarrelRacing();

    /**
     * Follow the "slalom" path defined in the 2021 At-Home Challenges manual.
     */
    void AutoNavSlalom();

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
    ShootingState m_state = ShootingState::kIdle;
    int m_ballsToShoot = -1;
    units::second_t m_shootTimeout = kMaxShootTimeout;
    bool m_prevFlywheelAtGoal = false;
    frc2::Timer m_timer;

    nt::NetworkTableEntry m_LEDEntry =
        NetworkTableUtil::MakeBoolEntry("/photonvision/ledMode");

    AutonomousChooser m_autonChooser{"No-op", [=] { AutoNoOp(); }};

    frc::CSVLogFile m_batteryLogger{"Battery", "Battery voltage (V)"};
    frc::CSVLogFile m_eventLogger{"Events", "Event"};

    nt::NetworkTableEntry m_batteryVoltageEntry =
        NetworkTableUtil::MakeDoubleEntry("/Diagnostics/Robot/batteryVoltage");
    nt::NetworkTableEntry m_ballsToShootEntry =
        NetworkTableUtil::MakeDoubleEntry("/Diagnostics/Robot/ballsToShoot");
    nt::NetworkTableEntry m_autoGalacticSearchPath =
        NetworkTableUtil::MakeStringEntry(
            "/Diagnostics/Drivetrain/Outputs/Galactic Search Path", "none");
    nt::NetworkTableEntry m_autoGalacticSearchLayout =
        NetworkTableUtil::MakeStringEntry(
            "/Diagnostics/Drivetrain/Outputs/Galactic Search Layout", "none");
};

}  // namespace frc3512
