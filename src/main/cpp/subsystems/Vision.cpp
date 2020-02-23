// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Vision.hpp"

#include <chrono>
#include <vector>

#include <units/units.h>

using namespace frc3512;

Vision::Vision() : PublishNode("Vision") {
    m_inst.StartClient("10.35.12.2", 1735);
}

void Vision::TurnLEDOn() { m_ledIsOn.SetBoolean(true); }

void Vision::TurnLEDOff() { m_ledIsOn.SetBoolean(false); }

bool Vision::IsLEDOn() const { return !m_ledIsOn.GetBoolean(false); }

void Vision::SubsystemPeriodic() {
    namespace chrono = std::chrono;

    auto latency = static_cast<int64_t>(m_latency.GetDouble(-1) * 1000);

    // If valid latency data was received
    if (latency > 0) {
        std::vector<double> pose = m_pose.GetDoubleArray(-1);
        units::radian_t theta = units::degree_t{pose[2]};

        auto timestamp = chrono::system_clock::now().time_since_epoch();
        timestamp -= chrono::microseconds{latency};

        VisionPosePacket packet{
            "TargetPose", pose[0], pose[1], theta.to<double>(),
            chrono::duration_cast<chrono::microseconds>(timestamp).count()};
        Publish(packet);
    }
}

void Vision::ProcessMessage(const ButtonPacket& message) {
    if (message.topic == "Robot/AppendageStick2" && message.button == 1 &&
        message.pressed) {
        IsLEDOn();
    }
}

void Vision::ProcessMessage(const CommandPacket& message) {
    if (message.topic == "Robot/TeleopInit") {
        EnablePeriodic();
    }
}
