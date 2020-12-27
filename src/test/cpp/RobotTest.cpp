// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <gtest/gtest.h>

#include "Robot.hpp"
#include "SimulatorTest.hpp"

class RobotTest : public frc3512::SimulatorTest {};

// Make sure robot initializes
TEST_F(RobotTest, Init) { frc3512::Robot robot; }
