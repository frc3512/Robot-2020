// Copyright (c) 2018-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/controller/LinearPlantInversionFeedforward.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/system/LinearSystem.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/voltage.h>
#include <wpi/math>

#include "Constants.hpp"
#include "controllers/ControllerBase.hpp"

namespace frc3512 {

class FlywheelController : public ControllerBase<1, 1, 1> {
public:
    static constexpr auto kV = 0.009964_V / 1_rad_per_s;
    static constexpr auto kA = 0.00299_V / 1_rad_per_s_sq;

    static constexpr double kGearRatio = 8.0;
    static constexpr double kDpP = (wpi::math::pi * 2.0) * kGearRatio / 512.0;
    static constexpr auto kMaxAngularVelocity = 1000.5_rad_per_s;

    /**
     * Constructs a flywheel controller.
     */
    FlywheelController();

    FlywheelController(FlywheelController&&) = default;
    FlywheelController& operator=(FlywheelController&&) = default;

    /**
     * States of the flywheel system.
     */
    class State {
    public:
        static constexpr int kAngularVelocity = 0;
    };

    /**
     * Inputs of the flywheel system.
     */
    class Input {
    public:
        static constexpr int kVoltage = 0;
    };

    /**
     * Outputs of the flywheel system.
     */
    class Output {
    public:
        static constexpr int kAngularVelocity = 0;
    };

    /**
     * Sets the goal.
     *
     * @param goal Angular velocity in radians per second
     */
    void SetGoal(units::radians_per_second_t goal);

    units::radians_per_second_t GetGoal() const;

    /**
     * Returns whether or not the goal has been reached.
     */
    bool AtGoal() const;

    /**
     * Resets any internal state.
     */
    void Reset();

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
        m_plant, {20.0}, {12.0}, Constants::kDt};
    frc::LinearPlantInversionFeedforward<1, 1> m_ff{m_plant, Constants::kDt};

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
