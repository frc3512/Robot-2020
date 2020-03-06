// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/Joystick.h>
#include <frc/TimedRobot.h>
#include <frc2/Timer.h>

#include "Constants.hpp"
#include "autonselector/AutonSelector.hpp"
#include "communications/PublishNode.hpp"
#include "subsystems/Climber.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Flywheel.hpp"
#include "subsystems/Intake.hpp"
#include "subsystems/Turret.hpp"
#include "subsystems/Vision.hpp"

namespace frc3512 {

using namespace frc3512::Constants::Robot;

class Robot : public frc::TimedRobot, public PublishNode {
public:
    enum class ShootingState { kIdle, kStartFlywheel, kStartConveyor };

    Robot();

    void DisabledInit() override;
    void AutonomousInit() override;
    void TeleopInit() override;
    void TestInit() override;

    void RobotPeriodic() override;
    void DisabledPeriodic() override;
    void AutonomousPeriodic() override;
    void TeleopPeriodic() override;

    void AutoLoadingZoneDriveForwardInit();
    void AutoLoadingZoneShootThreeInit();
    void AutoTargetZoneShootThreeInit();
    void AutoRightSideShootThreeInit();

    void AutoLoadingZoneDriveForwardPeriodic();
    void AutoLoadingZoneShootThreePeriodic();
    void AutoTargetZoneShootThreePeriodic();
    void AutoRightSideShootThreePeriodic();

private:
    // The order the subsystems are initialized determines the order the
    // controllers run in.
    Drivetrain m_drivetrain;
    Turret m_turret{m_drivetrain};
    Flywheel m_flywheel{m_turret};
    Intake m_intake{m_flywheel};
    Vision m_vision;
    Climber m_climber;
    frc::Joystick m_driveStick1{kDriveStick1Port};
    frc::Joystick m_driveStick2{kDriveStick2Port};
    frc::Joystick m_appendageStick{kAppendageStickPort};
    frc::Joystick m_appendageStick2{kAppendageStick2Port};

    ShootingState m_state = ShootingState::kIdle;
    frc2::Timer m_timer;

    AutonSelector m_autonSelector{kDsPort};
};
}  // namespace frc3512
