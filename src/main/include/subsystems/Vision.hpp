// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <iostream>

#include <frc/geometry/Pose2d.h>
#include <networktables/NetworkTableInstance.h>

#include "communications/PublishNode.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

class Vision : public SubsystemBase, public PublishNode {
public:
    Vision();
    Vision& operator=(const Vision&) = delete;

    void ToggleLED();

    void SubsystemPeriodic() override;

    void ProcessMessage(const ButtonPacket& message) override;

    void ProcessMessage(const CommandPacket& message) override;

private:
    frc::Pose2d m_pose;
    units::radian_t m_yaw;
    
    double CameraPose, CameraYaw;
    nt::NetworkTableInstance inst{nt::NetworkTableInstance::GetDefault()};
};
}  // namespace frc3512
