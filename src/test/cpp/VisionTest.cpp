// Copyright (c) 2019-2020 FRC Team 3512. All Rights Reserved.

#include <gtest/gtest.h>
#include <networktables/NetworkTableInstance.h>

#include "TargetModel.hpp"
#include "subsystems/Vision.hpp"

#define EXPECT_NEAR_UNITS(val1, val2, eps) \
    EXPECT_LE(units::math::abs(val1 - val2), eps)

TEST(VisionTest, CalculateDrivetrainInGlobal) {
    frc3512::Vision vision;

    auto inst = nt::NetworkTableInstance::GetDefault();
    inst.StartLocal();
    auto poseTable = inst.GetTable("chameleon-vision");
    auto rpiTable = poseTable->GetSubTable("RPI-Cam");
    auto pose = rpiTable->GetEntry("target-Pose");

    auto testMeasurement = [&](units::inch_t x, units::inch_t y,
                               units::degree_t theta, units::inch_t globalX,
                               units::inch_t globalY) {
        pose.SetDoubleArray({x.to<double>(), y.to<double>(),
                             theta.to<double>()});  // in, in, deg
        inst.Flush();

        // Delay to ensure value propagates to tables
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

        auto globalMeasurement = vision.GetGlobalMeasurement();
        ASSERT_TRUE(globalMeasurement.has_value());
        EXPECT_NEAR_UNITS(globalMeasurement.value().pose.Translation().X(),
                          globalX, 0.05_m);
        EXPECT_NEAR_UNITS(globalMeasurement.value().pose.Translation().Y(),
                          globalY, 0.05_m);
    };

    frc::Translation2d kTargetCenter{TargetModel::kCenter.X(),
                                     TargetModel::kCenter.Y()};
    testMeasurement(
        5_m, 0_m, 0_deg,
        kTargetCenter.X() - 5_m +
            frc3512::Vision::kCameraInGlobalToTurretInGlobal.Translation().X(),
        kTargetCenter.Y() - 0_m +
            frc3512::Vision::kCameraInGlobalToTurretInGlobal.Translation().Y());
    testMeasurement(5_m, 0_m, 45_deg, 12.3935_m, 5.73816_m);
    testMeasurement(5.0_m, -2.0_m, -45.0_deg, 11.0871_m, 0.0813087_m);

    inst.StopLocal();
}
