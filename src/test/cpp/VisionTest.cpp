// Copyright (c) 2019-2020 FRC Team 3512. All Rights Reserved.

#include <gtest/gtest.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <units/math.h>

#include "SetCurrentPath.hpp"
#include "TargetModel.hpp"
#include "subsystems/Vision.hpp"

#define EXPECT_NEAR_UNITS(val1, val2, eps) \
    EXPECT_LE(units::math::abs(val1 - val2), eps)

TEST(VisionTest, CalculateDrivetrainInGlobal) {
    frc3512::SetCurrentPath testPath{"VisionTest/CalculateDrivetrainInGlobal"};
    frc3512::Vision vision;

    auto inst = nt::NetworkTableInstance::GetDefault();
    auto pose = inst.GetEntry("chameleon-vision/RPI-Cam/target-Pose");

    auto testMeasurement = [&](units::inch_t x, units::inch_t y,
                               units::degree_t theta, units::inch_t globalX,
                               units::inch_t globalY) {
        pose.SetDoubleArray({x.to<double>(), y.to<double>(),
                             theta.to<double>()});  // in, in, deg
        nt::NetworkTableInstance::GetDefault().Flush();

        // Delay to ensure value propagates to tables
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        auto globalMeasurement = vision.GetGlobalMeasurement();
        ASSERT_TRUE(globalMeasurement.has_value());
        EXPECT_NEAR_UNITS(globalMeasurement.value().pose.X(), globalX, 0.05_m);
        EXPECT_NEAR_UNITS(globalMeasurement.value().pose.Y(), globalY, 0.05_m);
    };

    frc::Translation2d kTargetCenter{TargetModel::kCenter.X(),
                                     TargetModel::kCenter.Y()};
    testMeasurement(5_m, 0_m, 0_deg,
                    kTargetCenter.X() - 5_m +
                        frc3512::Vision::kCameraInGlobalToTurretInGlobal.X(),
                    kTargetCenter.Y() - 0_m +
                        frc3512::Vision::kCameraInGlobalToTurretInGlobal.Y());
    testMeasurement(5_m, 0_m, 45_deg, 12.3935_m, 5.73816_m);
    testMeasurement(5_m, -2_m, -45_deg, 11.0871_m, 0.0813087_m);
}
