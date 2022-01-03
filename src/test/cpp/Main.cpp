// Copyright (c) FRC Team 3512. All Rights Reserved.

#include <gtest/gtest.h>
#include <hal/HAL.h>

#include "logging/CSVUtil.hpp"

int main(int argc, char** argv) {
    frc3512::DeleteCSVs();

    HAL_Initialize(500, 0);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
