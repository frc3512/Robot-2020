// Copyright (c) 2018-2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/controller/LinearQuadraticRegulator.h>
#include <frc/estimator/KalmanFilter.h>
#include <frc/logging/CSVLogFile.h>
#include <frc/system/plant/LinearSystemId.h>
#include <units/units.h>
#include <wpi/math>

#include "Constants.hpp"
#include "controllers/ControllerBase.hpp"

namespace frc3512 {

class FlywheelController : public ControllerBase<1, 1, 1> {
public:
    static constexpr double kGearRatio = 8.0;
    static constexpr double kDpP = (wpi::math::pi * 2.0) * kGearRatio / 512.0;
    static constexpr units::radians_per_second_t kMaxAngularVelocity =
        1000.5_rad_per_s;

    /**
     * Constructs a flywheel controller.
     */
    FlywheelController();

    FlywheelController(const FlywheelController&) = delete;
    FlywheelController& operator=(const FlywheelController&) = delete;

    class State {
    public:
        static constexpr int kAngularVelocity = 0;
    };

    class Input {
    public:
        static constexpr int kVoltage = 0;
    };

    class Output {
    public:
        static constexpr int kAngularVelocity = 0;
    };

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

    const Eigen::Matrix<double, 1, 1>& GetReferences() const override;

    const Eigen::Matrix<double, 1, 1>& GetStates() const override;

    const Eigen::Matrix<double, 1, 1>& GetInputs() const override;

    const Eigen::Matrix<double, 1, 1>& GetOutputs() const override;

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
    static constexpr decltype(1_V / 1_rad_per_s) kV = 0.011_V / 1_rad_per_s;
    static constexpr decltype(1_V / (1_rad_per_s / 1_s)) kA =
        0.005515_V / (1_rad_per_s / 1_s);
    static constexpr units::radians_per_second_t kAngularVelocityTolerance =
        7.0_rad_per_s;

    // The current sensor measurements.
    Eigen::Matrix<double, 1, 1> m_y;

    frc::LinearSystem<1, 1, 1> m_plant =
        frc::IdentifyVelocitySystem(kV.to<double>(), kA.to<double>());

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
        m_plant, Constants::kDt, {700.0}, {50.0}};

    frc::LinearQuadraticRegulator<1, 1> m_lqr{
        m_plant, {80.0}, {12.0}, Constants::kDt};

    // Controller reference
    Eigen::Matrix<double, 1, 1> m_r;
    Eigen::Matrix<double, 1, 1> m_nextR;

    bool m_atGoal = false;
    bool m_isEnabled = false;

    // The loggers that generates the comma separated value files
    frc::CSVLogFile velocityLogger{"Flywheel Velocities",
                                   "Measured Angular Velocity (rad/s)",
                                   "Estimated Angular Velocity (rad/s)",
                                   "Angular Velocity Reference (rad/s)"};
    frc::CSVLogFile voltageLogger{"Flywheel Voltages", "Controller Voltage (V)",
                                  "Battery Voltage (V)"};

    Eigen::Matrix<double, 1, 1> m_u;

    void ScaleCapU(Eigen::Matrix<double, 1, 1>* u);
};
}  // namespace frc3512
