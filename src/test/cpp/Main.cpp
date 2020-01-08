// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <gtest/gtest.h>
#include <hal/HAL.h>

int main(int argc, char** argv) {
    HAL_Initialize(500, 0);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
