// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include <adi/ADIS16470_IMU.h>
#include <adi/simulation/ADIS16470_IMUSim.h>
#include <frc/simulation/SimDeviceSim.h>
#include <wpi/SmallString.h>
#include <wpi/raw_ostream.h>

using namespace frc::sim;

ADIS16470_IMUSim::ADIS16470_IMUSim(const frc::ADIS16470_IMU& imu) {
    wpi::SmallString<128> fullname;
    wpi::raw_svector_ostream os(fullname);
    os << "Gyro:ADIS16470" << '[' << imu.GetPort() << ']';
    frc::sim::SimDeviceSim deviceSim{fullname.c_str()};
    m_simAngle = deviceSim.GetDouble("angle_x");
    m_simRate = deviceSim.GetDouble("rate_x");
    m_simAccelX = deviceSim.GetDouble("accel_x");
}

void ADIS16470_IMUSim::SetAngle(units::degree_t angle) {
    m_simAngle.Set(angle.to<double>());
}

void ADIS16470_IMUSim::SetRate(units::degrees_per_second_t rate) {
    m_simRate.Set(rate.to<double>());
}

void ADIS16470_IMUSim::SetAccelInstantX(
    units::meters_per_second_squared_t accel) {
    m_simAccelX.Set(accel.to<double>() / 9.8);
}
