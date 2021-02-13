// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include "FlywheelCharacterizationUtil.hpp"

#include <array>
#include <cmath>

#include <fmt/core.h>

#include "subsystems/Flywheel.hpp"

using namespace frc3512;

double FlywheelCharacterizationUtil::GetAutoSpeed() { return m_autoSpeed; }

void FlywheelCharacterizationUtil::SetStartTime() {
    m_startTime = frc2::Timer::GetFPGATimestamp().to<double>();
    m_counter = 0;
}

void FlywheelCharacterizationUtil::UpdateData(
    units::radian_t angle, units::radians_per_second_t angularVelocity) {
    TestSetAutoSpeed();
    m_now = frc2::Timer::GetFPGATimestamp().to<double>();
    m_battery = frc::RobotController::GetInputVoltage();
    m_motorVolts = m_battery * std::abs(m_priorAutoSpeed);
    m_leftMotorVolts = m_motorVolts;
    m_rightMotorVolts = m_motorVolts;
    m_autoSpeed = m_autoSpeedEntry.GetDouble(0);
    m_priorAutoSpeed = m_autoSpeed;
    m_leftEncoderDistance = angle.to<double>();
    m_leftEncoderRate = angularVelocity.to<double>();
    m_rightEncoderDistance = m_leftEncoderDistance;
    m_rightEncoderRate = m_leftEncoderRate;

    std::array<double, 10> dataArray;
    dataArray[0] = m_now;
    dataArray[1] = m_battery;
    dataArray[2] = m_autoSpeed;
    dataArray[3] = m_leftMotorVolts;
    dataArray[4] = m_rightMotorVolts;
    dataArray[5] = m_leftEncoderDistance;
    dataArray[6] = m_rightEncoderDistance;
    dataArray[7] = m_leftEncoderRate;
    dataArray[8] = m_rightEncoderRate;
    dataArray[9] = gyroAngle;

    for (unsigned int i = 0; i < dataArray.size(); i++) {
        m_entries.push_back(dataArray[i]);
    }
    m_counter++;
}

void FlywheelCharacterizationUtil::SendData() {
    for (size_t i = 0; i < m_entries.size(); i++) {
        m_data = m_data + std::to_string(m_entries[i]) + ", ";
    }
    double elapsedTime =
        frc2::Timer::GetFPGATimestamp().to<double>() - m_startTime;
    fmt::print("Collected {} samples in {} seconds\n", m_counter, elapsedTime);
    m_telemetryEntry.SetString(m_data);
    m_entries.clear();
    m_data = "";
}

std::vector<double>& FlywheelCharacterizationUtil::TestGetEntries() {
    return m_entries;
}

void FlywheelCharacterizationUtil::TestSetAutoSpeed() {
    m_testAutoSpeed = m_testAutoSpeed + 2.03;
    m_autoSpeedEntry.SetDouble(m_testAutoSpeed);
}
