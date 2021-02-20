// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {
void Robot::AutoNavSlalom() {
    // Initial Pose - Right against the Start Zone border
    const frc::Pose2d kInitialPose{1.08_m, 0.6_m, units::radian_t{0}};
    // Arch Poses - Robot moves on the curve path above markers D4 to D8
    const frc::Pose2d kTopArchLeft{4.174_m, 2.805_m, units::radian_t{0}};
    const frc::Pose2d kTopArchRight{5.433_m, 2.805_m, units::radian_t{0}};
    // Ring poses - Robot loops around marker D10
    const frc::Pose2d kRingBottom{7.643_m, 0.626_m, units::radian_t{0}};
    const frc::Pose2d kRingRight{8.706_m, 1.572_m,
                                 units::radian_t{(wpi::math::pi) / 2}};
    const frc::Pose2d kRingTop{7.62_m, 2.458_m, units::radian_t{wpi::math::pi}};
    // Bottom Arch poses - Robot moves on the curve path below markers D8 to D4
    const frc::Pose2d kBottomArchRight{6.04_m, 0.672_m,
                                       units::radian_t{wpi::math::pi}};
    const frc::Pose2d kBottomArchLeft{3.358_m, 0.672_m,
                                      units::radian_t{wpi::math::pi}};
    // End Pose - Right inside the Finish Zone
    const frc::Pose2d kEndPose{1.08_m, 2.434_m, units::radian_t{wpi::math::pi}};

    turret.SetControlMode(TurretController::ControlMode::kManual);
    drivetrain.Reset(kInitialPose);

    drivetrain.AddTrajectory({kInitialPose, kTopArchLeft, kTopArchRight,
                              kRingBottom, kRingRight, kRingTop,
                              kBottomArchRight, kBottomArchLeft, kEndPose});

    while (!drivetrain.AtGoal()) {
        if (!m_autonChooser.Suspend()) {
            return;
        }
    }
}

}  // namespace frc3512
