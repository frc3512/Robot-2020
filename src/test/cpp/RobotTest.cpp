// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <gtest/gtest.h>

#include "Robot.hpp"
#include "SetCurrentPath.hpp"

// Make sure robot initializes
TEST(RobotTest, Init) {
    frc3512::SetCurrentPath testPath{"RobotTest/Init"};
    frc3512::Robot robot;
}
