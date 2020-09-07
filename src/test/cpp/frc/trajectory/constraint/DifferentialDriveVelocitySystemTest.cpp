/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <Eigen/LU>
#include <wpi/MathExtras.h>

#include "frc/geometry/Pose2d.h"
#include "frc/system/LinearSystem.h"
#include "frc/system/plant/LinearSystemId.h"
#include "frc/trajectory/TestTrajectory.h"
#include "frc/trajectory/TrajectoryGenerator.h"
#include "frc/trajectory/constraint/DifferentialDriveVelocitySystemConstraint.h"
#include "gtest/gtest.h"

TEST(DifferentialDriveVelocitySystemTest, Constraint) {
  constexpr auto kMaxVoltage = 10_V;

  // Pick an unreasonably large kA to ensure the constraint has to do some work
  const frc::LinearSystem<2, 2, 2> system =
      frc::LinearSystemId::IdentifyDrivetrainSystem(1.0, 3.0, 1.0, 3.0);
  const frc::DifferentialDriveKinematics kinematics{0.5_m};
  auto config = frc::TrajectoryConfig(12_mps, 12_mps_sq);
  config.AddConstraint(frc::DifferentialDriveVelocitySystemConstraint(
      system, kinematics, kMaxVoltage));

  auto trajectory = frc::TestTrajectory::GetTrajectory(config);

  auto time = 0_s;
  auto dt = 20_ms;
  auto duration = trajectory.TotalTime();

  while (time < duration) {
    auto point = trajectory.Sample(time);
    time += dt;

    const frc::ChassisSpeeds chassisSpeeds{point.velocity, 0_mps,
                                           point.velocity * point.curvature};

    auto [left, right] = kinematics.ToWheelSpeeds(chassisSpeeds);

    auto x = frc::MakeMatrix<2, 1>(left.to<double>(), right.to<double>());

    // Not really a strictly-correct test as we're using the chassis accel
    // instead of the wheel accel, but much easier than doing it "properly" and
    // a reasonable check anyway
    auto xDot = frc::MakeMatrix<2, 1>(point.acceleration.to<double>(),
                                      point.acceleration.to<double>());

    auto u = system.B().inverse() * (xDot - system.A() * x);

    EXPECT_TRUE(((-kMaxVoltage.to<double>() - 0.5) <= u(0)) &&
                (u(0) <= (kMaxVoltage.to<double>() + 0.5)));
    EXPECT_TRUE(((-kMaxVoltage.to<double>() - 0.5) <= u(1)) &&
                (u(1) <= (kMaxVoltage.to<double>() + 0.5)));
  }
}

TEST(DifferentialDriveVelocitySystemTest, HighCurvature) {
  constexpr auto kMaxVoltage = 10_V;

  const frc::LinearSystem<2, 2, 2> system =
      frc::LinearSystemId::IdentifyDrivetrainSystem(1.0, 3.0, 1.0, 3.0);
  // Large trackwidth - need to test with radius of curvature less than half of
  // trackwidth
  const frc::DifferentialDriveKinematics kinematics{3_m};

  auto config = frc::TrajectoryConfig(12_fps, 12_fps_sq);
  config.AddConstraint(frc::DifferentialDriveVelocitySystemConstraint(
      system, kinematics, kMaxVoltage));

  EXPECT_NO_FATAL_FAILURE(frc::TrajectoryGenerator::GenerateTrajectory(
      frc::Pose2d{1_m, 0_m, frc::Rotation2d{90_deg}}, {},
      frc::Pose2d{0_m, 1_m, frc::Rotation2d{180_deg}}, config));

  config.SetReversed(true);

  EXPECT_NO_FATAL_FAILURE(frc::TrajectoryGenerator::GenerateTrajectory(
      frc::Pose2d{0_m, 1_m, frc::Rotation2d{180_deg}}, {},
      frc::Pose2d{1_m, 0_m, frc::Rotation2d{90_deg}}, config));
}
