// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <memory>

#include <frc/geometry/Pose2d.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "communications/PublishNode.hpp"
#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

class Vision : public SubsystemBase, public PublishNode {
public:
    Vision();
    Vision& operator=(const Vision&) = delete;

    void IsLEDOn();

    void IsLEDOff();

    void SubsystemPeriodic() override;

    void ProcessMessage(const ButtonPacket& message) override;

    void ProcessMessage(const CommandPacket& message) override;

private:
    nt::NetworkTableInstance m_inst{nt::NetworkTableInstance::GetDefault()};
    std::shared_ptr<nt::NetworkTable> m_ledTable =
        m_inst.GetTable("LED Ring Light");
    nt::NetworkTableEntry m_ledIsOn = m_ledTable->GetEntry("LED-State");
    std::shared_ptr<nt::NetworkTable> m_poseTable =
        m_inst.GetTable("chameleon-vision");
    std::shared_ptr<nt::NetworkTable> m_rpiTable =
        m_poseTable->GetSubTable("RPI-Cam");
    nt::NetworkTableEntry m_pose = m_rpiTable->GetEntry("target-Pose");
    nt::NetworkTableEntry m_latency = m_rpiTable->GetEntry("latency");
};
}  // namespace frc3512