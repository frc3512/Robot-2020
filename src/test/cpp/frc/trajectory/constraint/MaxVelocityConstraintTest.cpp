/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <vector>

#include <units/time.h>
#include <units/velocity.h>

#include "frc/kinematics/DifferentialDriveKinematics.h"
#include "frc/trajectory/TestTrajectory.h"
#include "frc/trajectory/constraint/MaxVelocityConstraint.h"
#include "gtest/gtest.h"

using namespace frc;

TEST(MaxVelocityConstraintTest, Constraint) {
  constexpr auto maxVelocity = 1_fps;
  constexpr auto dt = 20_ms;

  auto config = TrajectoryConfig(13_fps, 13_fps_sq);
  config.AddConstraint(MaxVelocityConstraint(maxVelocity));

  auto trajectory = TestTrajectory::GetTrajectory(config);

  for (auto time = 0_s; time < trajectory.TotalTime(); time += dt) {
    const Trajectory::State point = trajectory.Sample(time);
    EXPECT_TRUE(units::math::abs(point.velocity) < maxVelocity + 0.05_mps);
  }
}
