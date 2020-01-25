// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "subsystems/Vision.hpp"

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>

#include <units/units.h>

using namespace frc3512;

Vision::Vision() : PublishNode("Vision") {
    inst.StartClient("10.35.12.2", 1735);
}

void Vision::ToggleLED() {
    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("LED Ring Light");
    bool LEDState = table->GetEntry("LED-State").GetBoolean(0);
    table->GetEntry("LED-State").SetBoolean(!LEDState);
    }

void Vision::SubsystemPeriodic() {
    auto table = inst.GetTable("chameleon-vision");
    auto subtable = table->GetSubTable("Microsoft LifeCam HD-3000");
    auto pose = subtable->GetEntry("pose").GetDoubleArray(0);
    units::radian_t theta = units::degree_t{pose[2]};
    m_pose =
        frc::Pose2d(units::meter_t{pose[0]}, units::meter_t{pose[1]}, theta);
    PosePacket packet{"TargetPose", pose[0], pose[1], theta.to<double>()};
    Publish(packet);
}

void Vision::ProcessMessage(const ButtonPacket& message) {
    if (message.topic == "Robot/AppendageStick" && message.button == 1 &&
        message.pressed) {  
            ToggleLED();
        }
}

void Vision::ProcessMessage(const CommandPacket& message) {
    if (message.topic == "Robot/TeleopInit") {
        EnablePeriodic();
    }
}
