// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>

#include "Constants.hpp"
#include "autonselector/AutonSelector.hpp"
#include "communications/PublishNode.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Flywheel.hpp"
#include "subsystems/Intake.hpp"
#include "subsystems/Turret.hpp"
#include "subsystems/Vision.hpp"

namespace frc3512 {

using namespace frc3512::Constants::Robot;

class Robot : public frc::TimedRobot, public PublishNode {
public:
    Robot();

    void DisabledInit() override;
    void AutonomousInit() override;
    void TeleopInit() override;
    void TestInit() override;

    void RobotPeriodic() override;
    void DisabledPeriodic() override;
    void AutonomousPeriodic() override;
    void TeleopPeriodic() override;

private:
    Drivetrain m_drivetrain;
    Flywheel m_flywheel;
    Turret m_turret;
    Intake m_intake;
    Vision m_vision;
    frc::Joystick m_driveStick1{kDriveStick1Port};
    frc::Joystick m_driveStick2{kDriveStick2Port};
    frc::Joystick m_appendageStick{kAppendageStickPort};
    frc::Joystick m_appendageStick2{kAppendageStick2Port};
};

}  // namespace frc3512
