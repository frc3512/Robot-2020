// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <memory>

#include <frc/geometry/Pose2d.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>

#include "subsystems/SubsystemBase.hpp"

namespace frc3512 {

class Vision : public SubsystemBase {
public:
    Vision();
    Vision& operator=(const Vision&) = delete;

    /**
     * Turns on power to the LED ring light.
     */
    void TurnLEDOn();

    /**
     * Turns off power to the LED ring light.
     */
    void TurnLEDOff();

    /**
     * Returns whether or not the LED ring light is on or off.
     */
    bool IsLEDOn() const;

    void RobotPeriodic() override;

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
