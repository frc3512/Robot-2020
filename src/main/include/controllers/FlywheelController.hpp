// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/controller/LinearPlantInversionFeedforward.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/system/LinearSystem.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/voltage.h>
#include <wpi/numbers>

#include "Constants.hpp"
#include "controllers/ControllerBase.hpp"

namespace frc3512 {

/**
 * The flywheel controller.
 *
 * The flywheel uses an LQR for feedback control and a plant inversion
 * feedforward to maintain steady-state velocity.
 */
class FlywheelController : public ControllerBase<1, 1, 1> {
public:
    /// Static friction system ID gain.
    static constexpr auto kS = 0.47564_V;

    /// Angular velocity system ID gain.
    static constexpr auto kV = 0.0088813_V / 1_rad_per_s;

    /// Angular acceleration system ID gain.
    static constexpr auto kA = 0.0045649_V / 1_rad_per_s_sq;

    /// Gear ratio from encoder to flywheel.
    static constexpr double kGearRatio = 8.0;

    /// Angle per encoder pulse.
    static constexpr double kDpP =
        (wpi::numbers::pi * 2.0) * kGearRatio / 512.0;

    /// Maximum flywheel angular velocity.
    static constexpr auto kMaxAngularVelocity = 12_V / kV;

    FlywheelController();

    /**
     * Move constructor.
     */
    FlywheelController(FlywheelController&&) = default;

    /**
     * Move assignment operator.
     */
    FlywheelController& operator=(FlywheelController&&) = default;

    /**
     * States of the flywheel system.
     */
    class State {
    public:
        /// Flywheel angular velocity.
        static constexpr int kAngularVelocity = 0;
    };

    /**
     * Inputs of the flywheel system.
     */
    class Input {
    public:
        /// Motor voltage.
        static constexpr int kVoltage = 0;
    };

    /**
     * Outputs of the flywheel system.
     */
    class Output {
    public:
        /// Flywheel angular velocity.
        static constexpr int kAngularVelocity = 0;
    };

    /**
     * Sets the goal.
     *
     * @param goal Angular velocity in radians per second
     */
    void SetGoal(units::radians_per_second_t goal);

    /**
     * Returns the goal.
     */
    units::radians_per_second_t GetGoal() const;

    /**
     * Returns whether or not the goal has been reached.
     */
    bool AtGoal() const;

    /**
     * Resets any internal state.
     */
    void Reset();

    /**
     * Returns the next output of the controller.
     *
     * @param x The current state x.
     */
    Eigen::Matrix<double, 1, 1> Calculate(
        const Eigen::Matrix<double, 1, 1>& x) override;

    /**
     * Returns the flywheel plant.
     */
    static frc::LinearSystem<1, 1, 1> GetPlant();

private:
    static constexpr auto kAngularVelocityShotThreshold = 25_rad_per_s;
    static constexpr auto kAngularVelocityRecoveryThreshold = 5_rad_per_s;

    frc::LinearSystem<1, 1, 1> m_plant{GetPlant()};
    frc::LinearQuadraticRegulator<1, 1> m_lqr{
        m_plant, {50.0}, {12.0}, Constants::kControllerPeriod};
    frc::LinearPlantInversionFeedforward<1, 1> m_ff{
        m_plant, Constants::kControllerPeriod};

    bool m_atGoal = false;

    /**
     * Update "at goal" flag based on next reference and current state estimate.
     *
     * This function applies hysteresis so AtGoal() doesn't chatter between true
     * and false.
     *
     * @param error The angular velocity error.
     */
    void UpdateAtGoal(units::radians_per_second_t error);
};
}  // namespace frc3512
