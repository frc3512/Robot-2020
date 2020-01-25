// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

namespace frc3512 {

Robot::Robot() : PublishNode("Robot") {
    m_vision.Subscribe(*this);
}

void Robot::DisabledInit() {}

void Robot::AutonomousInit() {}

void Robot::TeleopInit() {}

void Robot::TestInit() {}

void Robot::RobotPeriodic() {}

void Robot::DisabledPeriodic() {}

void Robot::AutonomousPeriodic() { TeleopPeriodic(); }

void Robot::TeleopPeriodic() {}
}  // namespace frc3512

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<frc3512::Robot>(); }
#endif
