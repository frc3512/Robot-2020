// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {
void Robot::AutoNavBounce() {
    // Initial Pose - right up against the Start zone border.
    const frc::Pose2d kInitialPose{1.076_m, 2.276_m, units::radian_t{0}};
    // First Star - right before the first star so the robot touches up against
    // it.
    const frc::Pose2d kFirstStar{2.303_m, 3.797_m,
                                 units::radian_t{(wpi::math::pi) / 2}};
    // First Bottom Curve - the first lower curve path between the fist and
    // second star.
    const frc::Pose2d kFirstBottomCurve{3.62_m, 0.806_m,
                                        units::radian_t{wpi::math::pi}};
    // Second Star - right before the second star so the robot touches up
    // against it.
    const frc::Pose2d kSecondStar{4.578_m, 3.797_m,
                                  units::radian_t{(3 * wpi::math::pi) / 2}};
    // Second Bottom Curve - the second lower curve path between the second and
    // third star.
    const frc::Pose2d kSecondBottomCurve{5.703_m, 0.844_m, units::radian_t{0}};
    // Third Star - right before the third star so the robot touches up against
    // it.
    const frc::Pose2d kThirdStar{6.815_m, 3.797_m,
                                 units::radian_t{(wpi::math::pi) / 2}};
    // End Pose - right inside the Finish Zone.
    const frc::Pose2d kEndPose{7.979_m, 2.263_m,
                               units::radian_t{wpi::math::pi}};

    turret.SetControlMode(TurretController::ControlMode::kManual);
    drivetrain.Reset(kInitialPose);

    auto forwardConfig = Drivetrain::MakeTrajectoryConfig();
    auto backwardConfig = Drivetrain::MakeTrajectoryConfig();
    backwardConfig.SetReversed(true);

    drivetrain.AddTrajectory(kInitialPose, {}, kFirstStar, forwardConfig);
    drivetrain.AddTrajectory({kFirstStar, kFirstBottomCurve, kSecondStar},
                             backwardConfig);
    drivetrain.AddTrajectory({kSecondStar, kSecondBottomCurve, kThirdStar},
                             forwardConfig);
    drivetrain.AddTrajectory(kThirdStar, {}, kEndPose, backwardConfig);

    while (!drivetrain.AtGoal()) {
        if (!m_autonChooser.Suspend()) {
            return;
        }
    }
}
}  // namespace frc3512
