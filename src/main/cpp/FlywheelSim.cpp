// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include "FlywheelSim.hpp"

#include <frc/system/plant/LinearSystemId.h>
#include <wpi/MathExtras.h>

using namespace frc3512;

FlywheelSim::FlywheelSim(const frc::LinearSystem<1, 1, 1>& plant,
                         const frc::DCMotor& gearbox, double gearing,
                         const std::array<double, 2>& measurementStdDevs)
    : frc::sim::LinearSystemSim<2, 1, 2>(
          [=] {
              return frc::LinearSystem<2, 1, 2>{
                  frc::MakeMatrix<2, 2>(0.0, 1.0, 0.0, plant.A(0, 0)),
                  frc::MakeMatrix<2, 1>(0.0, plant.B(0, 0)),
                  Eigen::Matrix<double, 2, 2>::Identity(),
                  frc::MakeMatrix<2, 1>(0.0, 0.0)};
          }(),
          measurementStdDevs),
      m_gearbox(gearbox),
      m_gearing(gearing) {}

FlywheelSim::FlywheelSim(const frc::DCMotor& gearbox, double gearing,
                         units::kilogram_square_meter_t moi,
                         const std::array<double, 2>& measurementStdDevs)
    : FlywheelSim(frc::LinearSystemId::FlywheelSystem(gearbox, moi, gearing),
                  gearbox, gearing, measurementStdDevs) {}

units::radian_t FlywheelSim::GetAngle() const {
    return units::radian_t{GetOutput(0)};
}

units::radians_per_second_t FlywheelSim::GetAngularVelocity() const {
    return units::radians_per_second_t{GetOutput(1)};
}

units::ampere_t FlywheelSim::GetCurrentDraw() const {
    // I = V / R - omega / (Kv * R)
    // Reductions are greater than 1, so a reduction of 10:1 would mean the
    // motor is spinning 10x faster than the output.
    return m_gearbox.Current(GetAngularVelocity() * m_gearing,
                             units::volt_t{m_u(0)}) *
           wpi::sgn(m_u(0));
}

void FlywheelSim::SetInputVoltage(units::volt_t voltage) {
    SetInput(frc::MakeMatrix<1, 1>(voltage.to<double>()));
}
