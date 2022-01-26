// Copyright (c) FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <wpi/numbers>

#include "Robot.hpp"

namespace frc3512 {
void Robot::AutoNavSlalom() {
    auto kMaxV = DrivetrainTrajectoryController::kMaxV * 0.3;

    // Initial Pose - Right against the Start Zone border
    const frc::Pose2d kInitialPose{1.08_m, 0.6_m, units::radian_t{0}};
    // Arch Poses - Robot moves on the curve path above markers D4 to D8
    const frc::Pose2d kTopArchLeft{2.500_m, 1.996_m,
                                   units::radian_t{wpi::numbers::pi / 3}};
    const frc::Pose2d kTopArchMiddle{4.655_m, 2.450_m, units::radian_t{0}};
    const frc::Pose2d kTopArchRight{5.733_m - 1_ft, 2.450_m,
                                    units::radian_t{0}};
    const frc::Pose2d kArchTopToLoop{6.913_m - 1_ft, 1.484_m,
                                     units::radian_t{3 * wpi::numbers::pi / 2}};
    // Ring poses - Robot loops around marker D10
    const frc::Pose2d kRingBottom{
        7.327_m + 1_ft, 0.743_m,
        units::radian_t{0}};  // more towards the middle
    const frc::Pose2d kRingRight{7.800_m + 8_in, 1.572_m + 1_ft,
                                 units::radian_t{wpi::numbers::pi / 2}};
    const frc::Pose2d kRingTop{7.62_m - 5_in, 2.458_m, 165_deg};
    const frc::Pose2d kRingToArch{6.7_m - 5_in, 1.497_m, 270_deg};
    // Bottom Arch poses - Robot moves on the curve path below markers D8 to D4
    const frc::Pose2d kBottomArchRight{6.04_m - 5_in, 0.672_m,
                                       units::radian_t{wpi::numbers::pi}};
    const frc::Pose2d kBottomArchLeft{3.358_m - 10_in, 0.672_m,
                                      units::radian_t{wpi::numbers::pi}};
    // End Pose - Right inside the Finish Zone
    const frc::Pose2d kEndPose{1.08_m - 5_in, 2.434_m - 5_in,
                               units::radian_t{wpi::numbers::pi}};

    /* Poses before fudging
    // Initial Pose - Right against the Start Zone border
    const frc::Pose2d kInitialPose{1.08_m, 0.6_m, units::radian_t{0}};
    // Arch Poses - Robot moves on the curve path above markers D4 to D8
    const frc::Pose2d kTopArchLeft{4.174_m, 2.805_m, units::radian_t{0}};
    const frc::Pose2d kTopArchRight{5.433_m, 2.805_m, units::radian_t{0}};
    // Ring poses - Robot loops around marker D10
    const frc::Pose2d kRingBottom{7.643_m, 0.626_m, units::radian_t{0}};
    const frc::Pose2d kRingRight{8.706_m, 1.572_m,
                                 units::radian_t{(wpi::numbers::pi) / 2}};
    const frc::Pose2d kRingTop{7.62_m, 2.458_m,
                               units::radian_t{wpi::numbers::pi}};
    // Bottom Arch poses - Robot moves on the curve path below markers D8 to D4
    const frc::Pose2d kBottomArchRight{6.04_m, 0.672_m,
                                       units::radian_t{wpi::numbers::pi}};
    const frc::Pose2d kBottomArchLeft{3.358_m, 0.672_m,
                                      units::radian_t{wpi::numbers::pi}};
    // End Pose - Right inside the Finish Zone
    const frc::Pose2d kEndPose{1.08_m, 2.434_m,
                               units::radian_t{wpi::numbers::pi}};

    turret.SetControlMode(TurretController::ControlMode::kManual);
    drivetrain.Reset(kInitialPose);

    auto config = Drivetrain::MakeTrajectoryConfig();
    config.AddConstraint(frc::MaxVelocityConstraint(kMaxV));

    drivetrain.AddTrajectory({kInitialPose, kTopArchLeft, kTopArchRight,
                              kRingBottom, kRingRight, kRingTop,
                              kBottomArchRight, kBottomArchLeft, kEndPose});
    */
    turret.SetControlMode(TurretController::ControlMode::kManual);
    drivetrain.Reset(kInitialPose);

    auto config = Drivetrain::MakeTrajectoryConfig();
    config.AddConstraint(frc::MaxVelocityConstraint(kMaxV));

    drivetrain.AddTrajectory(
        {kInitialPose, kTopArchLeft, kTopArchMiddle, kTopArchRight,
         kArchTopToLoop, kRingBottom, kRingRight, kRingTop, kRingToArch,
         kBottomArchRight, kBottomArchLeft, kEndPose},
        config);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }
}

}  // namespace frc3512
