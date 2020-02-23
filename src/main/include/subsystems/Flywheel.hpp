// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <mutex>

#include <frc/CounterBase.h>
#include <frc/Encoder.h>
#include <frc/RTNotifier.h>
#include <rev/CANSparkMax.h>

#include "Constants.hpp"
#include "LinearTable.hpp"
#include "communications/PublishNode.hpp"
#include "controllers/FlywheelController.hpp"
#include "controllers/TurretController.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

class Flywheel : public SubsystemBase, public PublishNode {
public:
    Flywheel();
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
     * Runs the control loop every 0.005 seconds.
     */
    void Enable();

    /**
     * Disables the notifier running the control loop.
     */
    void Disable();

    /**
     * Sets the goal of the controller.
     *
     * @param velocity The goal to pass to the controller in radians per second.
     */
    void SetGoal(units::radians_per_second_t velocity);

    /**
     * Takes the projected distance of the flywheel to the target and sets an
     * angular velocity goal determined by the lookup table
     */
    void Shoot();

    /**
     * Returns whether or not the controller has reached its goal.
     */
    bool AtGoal();

    /**
     * Updates the controller from sensors and the motors from the controller.
     */
    void Iterate();

    /**
     * Resets sensors and the controller.
     */
    void Reset();

    void ProcessMessage(const CommandPacket& message) override;

    void ProcessMessage(const TurretPosePacket& message) override;

private:
    const frc::Translation3d targetModelCenter =
        (TurretController::A + TurretController::G) / 2.0;
    const frc::Pose2d targetPoseInGlobal{targetModelCenter.X(),
                                         targetModelCenter.Y(),
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
    frc::RTNotifier m_thread{Constants::kControllerPrio, &Flywheel::Iterate,
                             this};
    std::chrono::steady_clock::time_point m_lastTime = [] {
        return std::chrono::steady_clock::now();
    }();
    frc::Pose2d m_nextTurretPose;
    std::mutex m_poseDataMutex;

    units::meter_t DistanceToTarget(const frc::Pose2d& nextPose);
};

}  // namespace frc3512
