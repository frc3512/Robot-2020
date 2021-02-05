// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Encoder.h>
#include <frc/LinearFilter.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/simulation/LinearSystemSim.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc2/Timer.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/voltage.h>

#include "Constants.hpp"
#include "LerpTable.hpp"
#include "NetworkTableUtil.hpp"
#include "controllers/FlywheelController.hpp"
#include "rev/CANSparkMax.hpp"
#include "subsystems/ControlledSubsystemBase.hpp"

namespace frc3512 {

class Drivetrain;

/**
 * Flywheel subsystem.
 */
class Flywheel : public ControlledSubsystemBase<1, 1, 1> {
public:
    explicit Flywheel(Drivetrain& drivetrain);

    Flywheel(const Flywheel&) = delete;
    Flywheel& operator=(const Flywheel&) = delete;

    /**
     * Returns angular displacement of the flywheel
     *
     * @return angular displacement in radians
     */

    units::radian_t GetAngle();

    /**
     * Returns angular velocity of the flywheel.
     *
     * @return angular velocity in radians per second
     */
    units::radians_per_second_t GetAngularVelocity() const;

    /**
     * Sets the goal of the controller.
     *
     * @param velocity The goal to pass to the controller in radians per second.
     */
    void SetGoal(units::radians_per_second_t velocity);

    /**
     * Returns the current goal of the controller.
     */
    units::radians_per_second_t GetGoal() const;

    /**
     * Returns true if the flywheel has reached the goal angular velocity.
     */
    bool AtGoal();

    /**
     * Takes the projected distance of the flywheel to the target and sets an
     * angular velocity goal determined by the lookup table
     */
    void SetGoalFromPose();

    /**
     * Returns true if the flywheel has been set to a nonzero goal.
     */
    bool IsOn() const;

    /**
     * Returns true if flywheel is spinning and it has reached the goal angular
     * velocity.
     */
    bool IsReady();

    /**
     * Resets sensors and the controller.
     */
    void Reset();

    /**
     * Returns current drawn in simulation.
     */
    units::ampere_t GetCurrentDraw() const;

    /**
     * Returns angular velocity reference that will hit the target at a given
     * drivetrain pose.
     *
     * @param drivetrainPose Drivetrain pose.
     */
    units::radians_per_second_t GetReferenceForPose(
        const frc::Pose2d& drivetrainPose) const;

    void DisabledInit() override { Disable(); }

    void AutonomousInit() override { Enable(); }

    void TeleopInit() override { Enable(); }

    void RobotPeriodic() override;

    void ControllerPeriodic() override;

    /**
     * Sets the simulation model's angular velocity.
     *
     * This is useful for simulating balls exiting the shooter.
     *
     * @param velocity The simulation's angular velocity.
     */
    void SetSimAngularVelocity(units::radians_per_second_t velocity);

private:
    static const frc::Pose2d kTargetPoseInGlobal;

    LerpTable<units::meter_t, units::radians_per_second_t> m_table;

    rev::CANSparkMax m_leftGrbx{Constants::Flywheel::kLeftPort,
                                rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_rightGrbx{Constants::Flywheel::kRightPort,
                                 rev::CANSparkMax::MotorType::kBrushless};
    frc::Encoder m_encoder{Constants::Flywheel::kEncoderA,
                           Constants::Flywheel::kEncoderB};

    frc::LinearSystem<1, 1, 1> m_plant{FlywheelController::GetPlant()};
    frc::KalmanFilter<1, 1, 1> m_observer{
        m_plant,
        {700.0},
        {FlywheelController::kDpP / Constants::kDt.to<double>()},
        Constants::kDt};
    FlywheelController m_controller;
    units::radian_t m_angle;
    units::radian_t m_lastAngle;
    units::second_t m_time = frc2::Timer::GetFPGATimestamp();
    units::second_t m_lastTime = m_time - Constants::kDt;

    // Filters out encoder quantization noise
    units::radians_per_second_t m_angularVelocity = 0_rad_per_s;
    frc::LinearFilter<units::radians_per_second_t> m_velocityFilter =
        frc::LinearFilter<units::radians_per_second_t>::MovingAverage(4);

    Drivetrain& m_drivetrain;

    // Used in test mode for manually setting flywheel goal. This is helpful for
    // measuring flywheel lookup table values.
    double m_testThrottle = 0.0;

    nt::NetworkTableEntry m_angularVelocityRefEntry =
        NetworkTableUtil::MakeEntry(
            "/Diagnostics/Flywheel/References/Angular velocity", 0);
    nt::NetworkTableEntry m_angularVelocityStateEntry =
        NetworkTableUtil::MakeEntry(
            "/Diagnostics/Flywheel/States/Angular velocity", 0);
    nt::NetworkTableEntry m_isOnEntry =
        NetworkTableUtil::MakeEntry("/Diagnostics/Flywheel/IsOn", false);
    nt::NetworkTableEntry m_isReadyEntry =
        NetworkTableUtil::MakeEntry("/Diagnostics/Flywheel/IsReady", false);
    nt::NetworkTableEntry m_manualAngularVelocityReferenceEntry =
        NetworkTableUtil::MakeEntry(
            "/Diagnostics/Flywheel/Manual angular velocity reference", 0);

    // Measurement noise isn't added because the simulated encoder stores the
    // count as an integer, which already introduces quantization noise.
    frc::sim::FlywheelSim m_flywheelSim{m_controller.GetPlant(),
                                        frc::DCMotor::NEO(2), 1.0 / 2.0};
    frc::LinearSystem<2, 1, 1> m_flywheelPosition =
        frc::LinearSystemId::IdentifyPositionSystem<units::radian>(
            FlywheelController::kV, FlywheelController::kA);
    frc::sim::LinearSystemSim<2, 1, 1> m_flywheelPositionSim{
        m_flywheelPosition};
    frc::sim::EncoderSim m_encoderSim{m_encoder};

    /**
     * Sets the voltage of the flywheel motor.
     *
     * @param voltage The capped voltage to be set
     */
    void SetVoltage(units::volt_t voltage);

    /**
     * Computes an angular velocity reference from the provided joystick
     * throttle value.
     *
     * @param throttle Throttle value from -1 (forward) to 1 (back).
     */
    static units::radians_per_second_t ThrottleToReference(double throttle);
};

}  // namespace frc3512
