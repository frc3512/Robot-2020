// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/TimedRobot.h>

#include "Constants.hpp"

namespace frc3512 {

class Robot : public frc::TimedRobot {
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
};

}  // namespace frc3512
