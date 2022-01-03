// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/simulation/LinearSystemSim.h>
#include <frc/system/LinearSystem.h>
#include <frc/system/plant/DCMotor.h>
#include <units/angular_velocity.h>
#include <units/moment_of_inertia.h>

namespace frc3512 {

/**
 * Represents a simulated flywheel mechanism with angular position and velocity
 * states.
 */
class FlywheelSim : public frc::sim::LinearSystemSim<2, 1, 2> {
public:
    /**
     * Creates a simulated flywhel mechanism.
     *
     * @param plant              The linear system representing the flywheel.
     * @param gearbox            The type of and number of motors in the
     *                           flywheel gearbox.
     * @param gearing            The gearing of the flywheel (numbers greater
     *                           than 1 represent reductions).
     * @param measurementStdDevs The standard deviation of the measurement
     *                           noise.
     */
    FlywheelSim(const frc::LinearSystem<1, 1, 1>& plant,
                const frc::DCMotor& gearbox, double gearing,
                const std::array<double, 2>& measurementStdDevs = {0.0, 0.0});

    /**
     * Creates a simulated flywhel mechanism.
     *
     * @param gearbox            The type of and number of motors in the
     * flywheel gearbox.
     * @param gearing            The gearing of the flywheel (numbers greater
     * than 1 represent reductions).
     * @param moi                The moment of inertia of the flywheel.
     * @param measurementStdDevs The standard deviation of the measurement
     * noise.
     */
    FlywheelSim(const frc::DCMotor& gearbox, double gearing,
                units::kilogram_square_meter_t moi,
                const std::array<double, 2>& measurementStdDevs = {0.0, 0.0});

    /**
     * Returns the flywheel angle.
     *
     * @return The flywheel angle.
     */
    units::radian_t GetAngle() const;

    /**
     * Returns the flywheel angular velocity.
     *
     * @return The flywheel angular velocity.
     */
    units::radians_per_second_t GetAngularVelocity() const;

    /**
     * Returns the flywheel current draw.
     *
     * @return The flywheel current draw.
     */
    units::ampere_t GetCurrentDraw() const override;

    /**
     * Sets the input voltage for the flywheel.
     *
     * @param voltage The input voltage.
     */
    void SetInputVoltage(units::volt_t voltage);

private:
    frc::DCMotor m_gearbox;
    double m_gearing;
};

}  // namespace frc3512
