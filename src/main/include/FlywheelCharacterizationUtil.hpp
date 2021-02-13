// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <string>
#include <vector>

#include <frc/RobotController.h>
#include <frc2/Timer.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <units/angular_velocity.h>

#include "NetworkTableUtil.hpp"

namespace frc3512 {

class FlywheelCharacterizationUtil {
public:
    double GetAutoSpeed();

    void SetStartTime();

    void UpdateData(units::radian_t angle,
                    units::radians_per_second_t angularVelocity);
    void SendData();

    std::vector<double>& TestGetEntries();

private:
    double gyroAngle = 0.0;

    std::string m_data;
    std::vector<double> m_entries;

    double m_leftEncoderDistance;
    double m_leftEncoderRate;
    double m_rightEncoderDistance;
    double m_rightEncoderRate;

    double m_startTime = 0;
    double m_now = 0;
    int m_counter = 0;

    double m_autoSpeed = 0;
    double m_priorAutoSpeed = 0;

    double m_battery;
    double m_motorVolts;
    double m_leftMotorVolts;
    double m_rightMotorVolts;

    nt::NetworkTableEntry m_telemetryEntry =
        NetworkTableUtil::MakeStringEntry("/robot/telemetry", "");
    nt::NetworkTableEntry m_autoSpeedEntry =
        NetworkTableUtil::MakeDoubleEntry("/robot/autospeed", 0.0);

    // used to set the autospeed in tests.
    double m_testAutoSpeed;
    void TestSetAutoSpeed();
};
}  // namespace frc3512
