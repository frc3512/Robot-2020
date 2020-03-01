// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <Eigen/Core>
#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/logging/CSVLogFile.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/FlywheelSystem.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/units.h>

#include "Constants.hpp"

namespace frc3512 {

class FlywheelController {
public:
    /**
     * Constructs a flywheel controller with the given coefficients.
     *
     * @param Qelems The maximum desired error tolerance for each state.
     * @param Relems The maximum desired control effort for each input.
     * @param dt     Discretization timestep.
     */
    FlywheelController(const std::array<double, 1>& Qelems,
                       const std::array<double, 1>& Relems, units::second_t dt);

    FlywheelController(const FlywheelController&) = delete;
    FlywheelController& operator=(const FlywheelController&) = delete;

    void Enable();
    void Disable();
    bool IsEnabled() const;

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
     * Sets the current encoder measurement.
     *
     * @param angularVelocity Anglular velocity of the flywheel in radians.
     */
    void SetMeasuredAngularVelocity(
        units::radians_per_second_t angularVelocity);

    /**
     * Returns the estimated angular velocity.
     */
    units::radians_per_second_t EstimatedAngularVelocity() const;

    /**
     * Returns the error between the angular velocity goal and the angular
     * velocity estimate.
     */
    units::radians_per_second_t AngularVelocityError() const;

    /**
     * Returns the current angular velocity goal.
     */
    units::radians_per_second_t AngularVelocityGoal() const;

    /**
     * Returns the current controller voltage.
     */
    units::volt_t ControllerVoltage() const;

    /**
     * Returns the estimated controller voltage.
     */
    units::volt_t EstimatedControllerVoltage() const;

    /**
     * Returns the error between the controller voltage and the controller
     * voltage estimate
     */
    units::volt_t ControllerVoltageError() const;

    /**
     * Executes the control loop for a cycle.
     */
    void Update(units::second_t dt, units::second_t elapsedTime);

    /**
     * Returns the augmented version of the model with voltage error estimation
     */
    const frc::LinearSystem<2, 1, 1>& GetAugmentedPlant() const;

    /**
     * Resets any internal state.
     */
    void Reset();

private:
    // The current sensor measurements.
    Eigen::Matrix<double, 1, 1> m_y;

    /* frc::LinearSystem<1, 1, 1> m_plant = [=] {
        constexpr auto motor = frc::DCMotor::NEO(2);

        // Moment of inertia
        constexpr auto J = 0.032_kg_sq_m;

        // Gear ratio from motor to output shaft
        constexpr double G = 2.0;

        return frc::FlywheelSystem(motor, J, G);
    }(); */

    frc::LinearSystem<1, 1, 1> m_plant =
        frc::IdentifyVelocitySystem(Constants::Flywheel::kV.to<double>(),
                                    Constants::Flywheel::kA.to<double>());

    frc::LinearSystem<2, 1, 1> m_augmentedPlant = [=] {
        Eigen::Matrix<double, 2, 2> A;
        A.block<1, 1>(0, 0) = m_plant.A();
        A.block<1, 1>(0, 1) = m_plant.B();
        A.block<1, 2>(1, 0) = Eigen::Matrix<double, 1, 2>::Zero();
        Eigen::Matrix<double, 2, 1> B;
        B.block<1, 1>(0, 0) = m_plant.B();
        B(1, 0) = 0;
        Eigen::Matrix<double, 1, 2> C;
        C.block<1, 1>(0, 0) = m_plant.C();
        C(0, 1) = 0;
        Eigen::Matrix<double, 1, 1> D;
        D = m_plant.D();
        auto augmentedPlant = frc::LinearSystem<2, 1, 1>(
            A, B, C, D, m_plant.Umin(), m_plant.Umax());
        return augmentedPlant;
    }();

    frc::KalmanFilter<1, 1, 1> m_observer{
        m_plant, Constants::kDt, {10.0}, {Constants::Flywheel::kDpP + 0.175}};

    frc::LinearQuadraticRegulator<1, 1> m_lqr;

    // Controller reference
    Eigen::Matrix<double, 1, 1> m_r;
    Eigen::Matrix<double, 1, 1> m_nextR;

    bool m_atGoal = false;
    bool m_isEnabled = false;

    // The loggers that generates the comma separated value files
    frc::CSVLogFile velocityLogger{"FlywheelVelocities",
                                   "Measured Angular Velocity (rad/s)",
                                   "Estimated Angular Velocity (rad/s)",
                                   "Angular Velocity Reference (rad/s)"};
    frc::CSVLogFile voltageLogger{"FlywheelVoltages", "Controller Voltage (V)",
                                  "Voltage Error (V)", "Battery Voltage (V)"};

    Eigen::Matrix<double, 1, 1> m_u;

    void ScaleCapU(Eigen::Matrix<double, 1, 1>* u);
};
}  // namespace frc3512
