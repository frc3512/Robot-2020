// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <mutex>

#include <frc/CounterBase.h>
#include <frc/Encoder.h>
#include <frc/RTNotifier.h>
#include <rev/CANSparkMax.h>
#include <wpi/mutex.h>

#include "Constants.hpp"
#include "LinearTable.hpp"
#include "controllers/FlywheelController.hpp"
#include "subsystems/ControllerSubsystemBase.hpp"
#include "subsystems/Turret.hpp"

namespace frc3512 {

class Flywheel : public ControllerSubsystemBase {
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
    units::radians_per_second_t GetAngularVelocity();

    /**
     * Enables the controller.
     */
    void EnableController();

    /**
     * Disables the controller.
     */
    void DisableController();

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
     * Takes the projected distance of the flywheel to the target and sets an
     * angular velocity goal determined by the lookup table
     */
    bool AtGoal() const;

    /**
     * Takes the projected distance of the flywheel to the target and sets an
     * angular velocity goal determined by the lookup table
     */
    void Shoot();

    /**
     * Returns true if flywheel is spinning.
     */
    bool IsShooting() const;

    /**
     * Resets sensors and the controller.
     */
    void Reset();

    void DisabledInit() override { DisableController(); }

    void AutonomousInit() override { EnableController(); }

    void TeleopInit() override { EnableController(); }

    void ControllerPeriodic() override;

private:
    const frc::Pose2d kTargetPoseInGlobal{TargetModel::kCenter.X(),
                                          TargetModel::kCenter.Y(),
                                          units::radian_t{wpi::math::pi}};
    lookup::unbounded_linear_table<units::meter_t, units::radians_per_second_t>
        m_table;
    rev::CANSparkMax m_leftGrbx{Constants::Flywheel::kLeftPort,
                                rev::CANSparkMax::MotorType::kBrushless};
    rev::CANSparkMax m_rightGrbx{Constants::Flywheel::kRightPort,
                                 rev::CANSparkMax::MotorType::kBrushless};
    frc::Encoder m_encoder{Constants::Flywheel::kEncoderA,
                           Constants::Flywheel::kEncoderB};
    FlywheelController m_controller{{80.0}, {12.0}, Constants::kDt};
    std::chrono::steady_clock::time_point m_lastTime =
        std::chrono::steady_clock::now();
    frc::Pose2d m_nextTurretPose;
    mutable wpi::mutex m_controllerMutex;

    Turret& m_turret;
};

}  // namespace frc3512
