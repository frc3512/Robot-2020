// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/geometry/Rotation2d.h>
#include <hal/SimDevice.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

namespace frc {

class ADIS16470_IMU;

namespace sim {

/**
 * Class to control a simulated ADIS16470 IMU.
 */
class ADIS16470_IMUSim {
public:
    /**
     * Constructs from a ADIS16470_IMU object.
     *
     * @param imu ADIS16470_IMU to simulate
     */
    explicit ADIS16470_IMUSim(const ADIS16470_IMU& imu);

    /**
     * Sets the angle.
     *
     * @param angle The angle (clockwise positive).
     */
    void SetAngle(units::degree_t angle);

    /**
     * Sets the angular rate (clockwise positive).
     *
     * @param rate The angular rate.
     */
    void SetRate(units::degrees_per_second_t rate);

    /**
     * Sets the X axis acceleration.
     *
     * @param accel The acceleration.
     */
    void SetAccelInstantX(units::meters_per_second_squared_t accel);

    /**
     * Sets the Y axis acceleration.
     *
     * @param accel The acceleration.
     */
    void SetAccelInstantY(units::meters_per_second_squared_t accel);

private:
    hal::SimDouble m_simAngle;
    hal::SimDouble m_simRate;
    hal::SimDouble m_simAccelX;
    hal::SimDouble m_simAccelY;
};

}  // namespace sim
}  // namespace frc
