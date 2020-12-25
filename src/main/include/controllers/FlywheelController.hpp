// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/controller/LinearPlantInversionFeedforward.h>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/system/plant/LinearSystemId.h>
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
    static constexpr auto kV = 0.011_V / 1_rad_per_s;
    static constexpr auto kA = 0.005515_V / 1_rad_per_s_sq;

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

    const Eigen::Matrix<double, 1, 1>& GetReferences() const override;

    const Eigen::Matrix<double, 1, 1>& GetStates() const override;

    /**
     * Resets any internal state.
     */
    void Reset();

    Eigen::Matrix<double, 1, 1> Update(const Eigen::Matrix<double, 1, 1>& y,
                                       units::second_t dt) override;

    /**
     * Returns the flywheel plant.
     */
    const frc::LinearSystem<1, 1, 1>& GetPlant() const;

private:
    static constexpr auto kAngularVelocityTolerance = 20.0_rad_per_s;

    frc::LinearSystem<1, 1, 1> m_plant =
        frc::LinearSystemId::IdentifyVelocitySystem<units::radian>(kV, kA);
    frc::LinearQuadraticRegulator<1, 1> m_lqr{
        m_plant, {80.0}, {12.0}, Constants::kDt};
    frc::LinearPlantInversionFeedforward<1, 1> m_ff{m_plant, Constants::kDt};

    frc::KalmanFilter<1, 1, 1> m_observer{
        m_plant,
        {700.0},
        {FlywheelController::kDpP / Constants::kDt.to<double>()},
        Constants::kDt};

    // Controller reference
    Eigen::Matrix<double, 1, 1> m_r;
    Eigen::Matrix<double, 1, 1> m_nextR;

    bool m_atGoal = false;
};
}  // namespace frc3512
