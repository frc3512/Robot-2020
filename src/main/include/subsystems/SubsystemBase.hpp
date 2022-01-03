// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <wpi/SmallVector.h>

namespace frc3512 {

/**
 * A standardized base for subsystems.
 */
class SubsystemBase {
public:
    /**
     * Constructs a SubsystemBase.
     */
    SubsystemBase();

    /**
     * Copy constructor.
     */
    SubsystemBase(const SubsystemBase&) = default;

    /**
     * Copy assignment operator.
     */
    SubsystemBase& operator=(const SubsystemBase&) = default;

    /**
     * Move constructor.
     */
    SubsystemBase(SubsystemBase&&) = default;

    /**
     * Move assignment operator.
     */
    SubsystemBase& operator=(SubsystemBase&&) = default;

    virtual ~SubsystemBase();

    /**
     * Robot-wide simulation initialization code should go here.
     *
     * Users should override this method for default Robot-wide simulation
     * related initialization which will be called when the robot is first
     * started. It will be called exactly one time after RobotInit is called
     * only when the robot is in simulation.
     */
    virtual void SimulationInit() {}

    /**
     * Initialization code for disabled mode should go here.
     */
    virtual void DisabledInit() {}

    /**
     * Initialization code for autonomous mode should go here.
     */
    virtual void AutonomousInit() {}

    /**
     * Initialization code for teleop mode should go here.
     */
    virtual void TeleopInit() {}

    /**
     * Initialization coe for test mode should go here.
     */
    virtual void TestInit() {}

    /**
     * Periodic code for all modes should go here.
     */
    virtual void RobotPeriodic() {}

    /**
     * Periodic simulation code should go here.
     *
     * This function is called in a simulated robot after user code executes.
     */
    virtual void SimulationPeriodic() {}

    /**
     * Periodic code for disabled mode should go here.
     */
    virtual void DisabledPeriodic() {}

    /**
     * Periodic code for autonomous mode should go here.
     */
    virtual void AutonomousPeriodic() {}

    /**
     * Periodic code for teleop mode should go here.
     */
    virtual void TeleopPeriodic() {}

    /**
     * Periodic code for test mode should go here.
     */
    virtual void TestPeriodic() {}

    /**
     * Call all subsystems's SimulationInit().
     */
    static void RunAllSimulationInit();

    /**
     * Call all subsystems's DisabledInit().
     */
    static void RunAllDisabledInit();

    /**
     * Call all subsystems's AutonomousInit().
     */
    static void RunAllAutonomousInit();

    /**
     * Call all subsystems's TeleopInit().
     */
    static void RunAllTeleopInit();

    /**
     * Call all subsystems's TestInit().
     */
    static void RunAllTestInit();

    /**
     * Call all subsystems's RobotPeriodic().
     */
    static void RunAllRobotPeriodic();

    /**
     * Call all subsystems's SimulationPeriodic().
     */
    static void RunAllSimulationPeriodic();

    /**
     * Call all subsystems's DisabledPeriodic().
     */
    static void RunAllDisabledPeriodic();

    /**
     * Call all subsystems's AutonomousPeriodic().
     */
    static void RunAllAutonomousPeriodic();

    /**
     * Call all subsystems's TeleopPeriodic().
     */
    static void RunAllTeleopPeriodic();

    /**
     * Call all subsystems's TestPeriodic().
     */
    static void RunAllTestPeriodic();

private:
    static wpi::SmallVector<SubsystemBase*, 8> m_subsystems;

    /**
     * Consumes button edge events produced in disabled mode.
     */
    static void ConsumeButtonEdgeEvents();
};

}  // namespace frc3512
