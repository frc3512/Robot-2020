// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Encoder.h>
#include <frc/Timer.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/filter/LinearFilter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/simulation/LinearSystemSim.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/plant/LinearSystemId.h>
#include <rev/CANSparkMax.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/voltage.h>
#include <wpi/static_circular_buffer.h>

#include "Constants.hpp"
#include "FlywheelSim.hpp"
#include "HWConfig.hpp"
#include "LerpTable.hpp"
#include "controllers/FlywheelController.hpp"
#include "subsystems/ControlledSubsystemBase.hpp"
#include "subsystems/Flywheel.hpp"
#include "subsystems/Vision.hpp"

namespace frc3512 {

class Drivetrain;

/**
 * The flywheel subsystem.
 *
 * The flywheel uses a Kalman filter for state estimation.
 */
class Flywheel : public ControlledSubsystemBase<1, 1, 1> {
public:
    /**
     * Producer-consumer queue for global pose measurements from Vision
     * subsystem.
     */
    wpi::static_circular_buffer<Vision::GlobalMeasurement, 8> visionQueue;

    /**
     * Constructs a Flywheel.
     *
     * @param drivetrain Drivetrain subsystem.
     */
    explicit Flywheel(Drivetrain& drivetrain);

    Flywheel(const Flywheel&) = delete;
    Flywheel& operator=(const Flywheel&) = delete;

    /**
     * Sets whether the robot will adjust the flywheel's
     * radians while the robot is moving
     *
     * @param moveAndShoot Whether or not to move and shoot.
     */
    void SetMoveAndShoot(bool moveAndShoot);

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
    bool AtGoal() const;

    /**
     * Takes the projected distance of the flywheel to the target and sets an
     * angular velocity goal determined by the lookup table
     */
    void SetGoalFromPose();

    /**
     * Takes range measurement from vision subsystem. Range measurement is flat
     * distance from front of target to face of camera. Sets angular velocity
     * goal determined by the lookup table.
     */
    void SetGoalFromVision();

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
     */
    units::radians_per_second_t GetReferenceForPose(
        const frc::Pose2d& drivetrainPose) const;

    /**
     * Returns angular velocity reference that will hit the target at a given
     * range measurement from vision.
     */
    units::radians_per_second_t GetReferenceForRange(
        units::meter_t range) const;

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
    // LUT from range to target to flywheel angular velocity
    LerpTable<units::meter_t, units::radians_per_second_t> m_table;

    rev::CANSparkMax m_leftGrbx{HWConfig::Flywheel::kLeftMotorID,
                                rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_rightGrbx{HWConfig::Flywheel::kRightMotorID,
                                 rev::CANSparkMax::MotorType::kBrushless};
    frc::Encoder m_encoder{HWConfig::Flywheel::kEncoderA,
                           HWConfig::Flywheel::kEncoderB};

    frc::LinearSystem<1, 1, 1> m_plant{FlywheelController::GetPlant()};
    frc::KalmanFilter<1, 1, 1> m_observer{
        m_plant,
        {200.0},
        {FlywheelController::kDpP / Constants::kControllerPeriod.value()},
        Constants::kControllerPeriod};

    FlywheelController m_controller;
    Eigen::Vector<double, 1> m_u = Eigen::Vector<double, 1>::Zero();

    bool m_moveAndShoot = true;

    units::radian_t m_angle;
    units::radian_t m_lastAngle;
    units::second_t m_time = frc::Timer::GetFPGATimestamp();
    units::second_t m_lastTime = m_time - Constants::kControllerPeriod;

    units::meter_t m_distanceToTarget = 0_m;

    // Filters out encoder quantization noise
    units::radians_per_second_t m_angularVelocity = 0_rad_per_s;
    frc::LinearFilter<units::radians_per_second_t> m_velocityFilter =
        frc::LinearFilter<units::radians_per_second_t>::MovingAverage(4);

    Drivetrain& m_drivetrain;

    // Used in test mode for manually setting flywheel goal. This is helpful for
    // measuring flywheel lookup table values.
    double m_testThrottle = 0.0;

    // Measurement noise isn't added because the simulated encoder stores the
    // count as an integer, which already introduces quantization noise.
    FlywheelSim m_flywheelSim{m_controller.GetPlant(), frc::DCMotor::NEO(2),
                              1.0 / 2.0};
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
