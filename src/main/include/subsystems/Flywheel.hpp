// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Encoder.h>
#include <frc/LinearFilter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/simulation/FlywheelSim.h>
#include <frc/simulation/LinearSystemSim.h>
#include <frc/system/LinearSystem.h>
#include <frc2/Timer.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/voltage.h>
#include <wpi/math>

#include "Constants.hpp"
#include "LerpTable.hpp"
#include "TargetModel.hpp"
#include "controllers/FlywheelController.hpp"
#include "rev/CANSparkMax.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

class Turret;

class Flywheel : public SubsystemBase {
public:
    explicit Flywheel(Turret& turret);

    Flywheel(Flywheel&&) = default;
    Flywheel& operator=(Flywheel&&) = default;

    /**
     * Sets the voltage of the flywheel motor.
     *
     * @param voltage The capped voltage to be set
     */
    void SetVoltage(units::volt_t voltage);

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
    void Shoot();

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

    void DisabledInit() override {
        m_controller.Disable();
        m_controller.SetClosedLoop(false);
    }

    void AutonomousInit() override {
        m_controller.Enable();
        m_controller.SetClosedLoop(true);
    }

    void TeleopInit() override {
        m_controller.Enable();
        m_controller.SetClosedLoop(true);
    }

    void RobotPeriodic() override;

    void ControllerPeriodic();

private:
    const frc::Pose2d kTargetPoseInGlobal{TargetModel::kCenter.X(),
                                          TargetModel::kCenter.Y(),
                                          units::radian_t{wpi::math::pi}};

    LerpTable<units::meter_t, units::radians_per_second_t> m_table;

    rev::CANSparkMax m_leftGrbx{Constants::Flywheel::kLeftPort,
                                rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_rightGrbx{Constants::Flywheel::kRightPort,
                                 rev::CANSparkMax::MotorType::kBrushless};
    frc::Encoder m_encoder{Constants::Flywheel::kEncoderA,
                           Constants::Flywheel::kEncoderB};

    FlywheelController m_controller;
    units::radian_t m_angle;
    units::radian_t m_lastAngle;
    units::second_t m_time = frc2::Timer::GetFPGATimestamp();
    units::second_t m_lastTime = m_time - Constants::kDt;

    // Filters out encoder quantization noise
    units::radians_per_second_t m_angularVelocity = 0_rad_per_s;
    frc::LinearFilter<units::radians_per_second_t> m_velocityFilter =
        frc::LinearFilter<units::radians_per_second_t>::MovingAverage(4);

    Turret& m_turret;

    nt::NetworkTableInstance m_inst = nt::NetworkTableInstance::GetDefault();
    nt::NetworkTableEntry m_angularVelocityRefEntry =
        m_inst.GetEntry("/Diagnostics/Flywheel/References/Angular velocity");
    nt::NetworkTableEntry m_angularVelocityStateEntry =
        m_inst.GetEntry("/Diagnostics/Flywheel/States/Angular velocity");
    nt::NetworkTableEntry m_isOnEntry =
        m_inst.GetEntry("/Diagnostics/Flywheel/IsOn");
    nt::NetworkTableEntry m_isReadyEntry =
        m_inst.GetEntry("/Diagnostics/Flywheel/IsReady");
    nt::NetworkTableEntry m_controllerEnabledEntry =
        m_inst.GetEntry("/Diagnostics/Flywheel/Controller enabled");

    // Measurement noise isn't added because the simulated encoder stores the
    // count as an integer, which already introduces quantization noise.
    frc::sim::FlywheelSim m_flywheelSim{m_controller.GetPlant(),
                                        frc::DCMotor::NEO(2), 1.0 / 2.0};
    frc::LinearSystem<2, 1, 1> m_flywheelPosition =
        frc::LinearSystemId::IdentifyPositionSystem(
            FlywheelController::kV.to<double>(),
            FlywheelController::kA.to<double>());
    frc::sim::LinearSystemSim<2, 1, 1> m_flywheelPositionSim{
        m_flywheelPosition};
    frc::sim::EncoderSim m_encoderSim{m_encoder};
};

}  // namespace frc3512
