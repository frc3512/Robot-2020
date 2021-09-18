// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <wpi/MathExtras.h>

#include "Eigen/QR"
#include "frc/geometry/Pose2d.h"
#include "frc/system/LinearSystem.h"
#include "frc/system/plant/LinearSystemId.h"
#include "frc/trajectory/TestTrajectory.h"
#include "frc/trajectory/TrajectoryGenerator.h"
#include "frc/trajectory/constraint/DifferentialDriveVelocitySystemConstraint.h"
#include "gtest/gtest.h"

// TODO: The constraint used in this test violates the max voltage, but not
// doing so would mean the only way to reach the max steady-state velocity for
// that voltage is open-loop exponential convergence. The constraint's
// optimization intent should be clarified so we can write a better test.
TEST(DifferentialDriveVelocitySystemTest, DISABLED_Constraint) {
  constexpr auto kDt = 20_ms;
  constexpr auto kMaxVoltage = 10_V;

  // Pick an unreasonably large kA to ensure the constraint has to do some work
  const frc::LinearSystem<2, 2, 2> system =
      frc::LinearSystemId::IdentifyDrivetrainSystem(
          1_V / 1_mps, 3_V / 1_mps_sq, 1_V / 1_mps, 3_V / 1_mps_sq);
  const frc::DifferentialDriveKinematics kinematics{0.5_m};
  auto config = frc::TrajectoryConfig(12_mps, 12_mps_sq);
  config.AddConstraint(frc::DifferentialDriveVelocitySystemConstraint(
      system, kinematics, kMaxVoltage));

  auto trajectory = frc::TestTrajectory::GetTrajectory(config);

  auto time = 0_s;
  while (time < trajectory.TotalTime()) {
    auto point = trajectory.Sample(time);
    time += kDt;

    const frc::ChassisSpeeds chassisSpeeds{point.velocity, 0_mps,
                                           point.velocity * point.curvature};

    auto [left, right] = kinematics.ToWheelSpeeds(chassisSpeeds);

    Eigen::Vector<double, 2> x{left.value(), right.value()};

    // Not really a strictly-correct test as we're using the chassis accel
    // instead of the wheel accel, but much easier than doing it "properly" and
    // a reasonable check anyway
    Eigen::Vector<double, 2> xDot{point.acceleration.value(),
                                  point.acceleration.value()};

    Eigen::Vector<double, 2> u =
        system.B().householderQr().solve(xDot - system.A() * x);

    EXPECT_GE(u(0), -kMaxVoltage.value() - 0.5);
    EXPECT_LE(u(0), kMaxVoltage.value() + 0.5);
    EXPECT_GE(u(1), -kMaxVoltage.value() - 0.5);
    EXPECT_LE(u(1), kMaxVoltage.value() + 0.5);
  }
}

TEST(DifferentialDriveVelocitySystemTest, HighCurvature) {
  constexpr auto kMaxVoltage = 10_V;

  const frc::LinearSystem<2, 2, 2> system =
      frc::LinearSystemId::IdentifyDrivetrainSystem(
          1_V / 1_mps, 3_V / 1_mps_sq, 1_V / 1_mps, 3_V / 1_mps_sq);
  // Large trackwidth - need to test with radius of curvature less than half of
  // trackwidth
  const frc::DifferentialDriveKinematics kinematics{3_m};

  frc::TrajectoryConfig config{12_fps, 12_fps_sq};
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
