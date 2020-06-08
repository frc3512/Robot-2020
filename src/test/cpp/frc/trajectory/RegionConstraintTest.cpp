/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <vector>

#include <units.h>

#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "frc/trajectory/TestTrajectory.h"
#include "frc/trajectory/constraint/MaxVelocityConstraint.h"
#include "frc/trajectory/constraint/RegionConstraint.h"
#include "gtest/gtest.h"

using namespace frc;

TEST(RegionConstraintTest, Constraint) {
  constexpr auto maxVelocity = 2_fps;
  constexpr auto dt = 20_ms;

  auto config = TrajectoryConfig(13_fps, 13_fps_sq);
  MaxVelocityConstraint maxVelConstraint(maxVelocity);
  RegionConstraint regionConstraint(frc::Translation2d{5_ft, 2.5_ft}, 10_ft,
                                    5_ft, maxVelConstraint,
                                    Rotation2d(180_deg));
  config.AddConstraint(regionConstraint);

  auto trajectory = TestTrajectory::GetTrajectory(config);

  bool exceededConstraintOutsideRegion = false;
  for (auto time = 0_s; time < trajectory.TotalTime(); time += dt) {
    const Trajectory::State point = trajectory.Sample(time);

    auto translation = point.pose.Translation();
    if (translation.X() < 10_ft && translation.Y() < 5_ft) {
      EXPECT_TRUE(units::math::abs(point.velocity) < maxVelocity + 0.05_mps);
    } else if (units::math::abs(point.velocity) >= maxVelocity + 0.05_mps) {
      exceededConstraintOutsideRegion = true;
    }
  }

  EXPECT_TRUE(exceededConstraintOutsideRegion);
}

TEST(RegionConstraintTest, IsPoseInRegion) {
  constexpr auto maxVelocity = 2_fps;
  MaxVelocityConstraint maxVelConstraint(maxVelocity);
  RegionConstraint regionConstraintNoRotation(frc::Translation2d{1_ft, 1_ft},
                                              2_ft, 4_ft, maxVelConstraint,
                                              Rotation2d(0_deg));
  EXPECT_FALSE(regionConstraintNoRotation.IsPoseInRegion(
      frc::Pose2d(2.1_ft, 1_ft, 0_rad)));

  RegionConstraint regionConstraintWithRotation(frc::Translation2d{1_ft, 1_ft},
                                                2_ft, 4_ft, maxVelConstraint,
                                                Rotation2d(90_deg));
  EXPECT_TRUE(regionConstraintWithRotation.IsPoseInRegion(
      frc::Pose2d(2.1_ft, 1_ft, 0_rad)));
}
