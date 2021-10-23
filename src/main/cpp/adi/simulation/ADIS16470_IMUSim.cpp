// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include <adi/ADIS16470_IMU.h>
#include <adi/simulation/ADIS16470_IMUSim.h>
#include <frc/simulation/SimDeviceSim.h>

using namespace frc::sim;

ADIS16470_IMUSim::ADIS16470_IMUSim(const frc::ADIS16470_IMU& imu) {
    frc::sim::SimDeviceSim deviceSim{"Gyro:ADIS16470", imu.GetPort()};
    m_simAngle = deviceSim.GetDouble("angle_x");
    m_simRate = deviceSim.GetDouble("rate_x");
    m_simAccelX = deviceSim.GetDouble("accel_x");
    m_simAccelY = deviceSim.GetDouble("accel_y");
}

void ADIS16470_IMUSim::SetAngle(units::degree_t angle) {
    m_simAngle.Set(angle.value());
}

void ADIS16470_IMUSim::SetRate(units::degrees_per_second_t rate) {
    m_simRate.Set(rate.value());
}

void ADIS16470_IMUSim::SetAccelInstantX(
    units::meters_per_second_squared_t accel) {
    m_simAccelX.Set(accel.value() / 9.81);
}

void ADIS16470_IMUSim::SetAccelInstantY(
    units::meters_per_second_squared_t accel) {
    m_simAccelY.Set(accel.value() / 9.81);
}
