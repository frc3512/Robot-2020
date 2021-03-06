// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {
void Robot::AutoNavBounce() {
    const auto kMarkerOffset = -Drivetrain::kLength / 16.0;

    // Initial Pose - right up against the Start zone border.
    const frc::Pose2d kInitialPose{1.076_m, 2.276_m, units::radian_t{0}};
    // First Star - right before the first star so the robot touches up against
    // it.
    const frc::Pose2d kFirstStar{2.303_m, 3.797_m + kMarkerOffset,
                                 units::radian_t{(wpi::math::pi) / 2}};
    // First Side Curve 1 - the first lower curve path between the fist and
    // second star.
    const frc::Pose2d kFirstSideCurve1{2.7_m + 0.15_m, 2.0 * 0.806_m,
                                       units::radian_t{wpi::math::pi / 2.0}};
    // First Bottom Curve - the first lower curve path between the fist and
    // second star.
    const frc::Pose2d kFirstBottomCurve1{3.62_m - 0.3_m, 0.806_m,
                                         units::radian_t{wpi::math::pi}};
    // First Bottom Curve - the first lower curve path between the fist and
    // second star.
    const frc::Pose2d kFirstBottomCurve2{3.62_m + 0.3_m, 0.806_m,
                                         units::radian_t{wpi::math::pi}};
    // First Side Curve 2 - the first lower curve path between the fist and
    // second star.
    const frc::Pose2d kFirstSideCurve2{4.578_m, 2.0_m,
                                       units::radian_t{3 * wpi::math::pi / 2}};
    // Second Star - right before the second star so the robot touches up
    // against it.
    const frc::Pose2d kSecondStar{4.578_m, 3.797_m + kMarkerOffset,
                                  units::radian_t{(3 * wpi::math::pi) / 2}};
    // Second Bottom Curve - the second lower curve path between the second and
    // third star.
    const frc::Pose2d kSecondBottomCurve{5.703_m, 0.844_m, units::radian_t{0}};
    // Third Star - right before the third star so the robot touches up against
    // it.
    const frc::Pose2d kThirdStar{6.815_m, 3.797_m + kMarkerOffset,
                                 units::radian_t{(wpi::math::pi) / 2}};
    // End Pose - right inside the Finish Zone.
    const frc::Pose2d kEndPose{7.979_m, 2.5_m, units::radian_t{wpi::math::pi}};

    m_turret.SetControlMode(TurretController::ControlMode::kManual);
    m_drivetrain.Reset(kInitialPose);

    auto forwardConfig = Drivetrain::MakeTrajectoryConfig();
    auto backwardConfig = Drivetrain::MakeTrajectoryConfig();
    backwardConfig.SetReversed(true);

    m_drivetrain.AddTrajectory(kInitialPose, {}, kFirstStar, forwardConfig);
    m_drivetrain.AddTrajectory(
        {kFirstStar, kFirstSideCurve1, kFirstBottomCurve1, kFirstBottomCurve2,
         kFirstSideCurve2, kSecondStar},
        backwardConfig);
    m_drivetrain.AddTrajectory({kSecondStar, kSecondBottomCurve, kThirdStar},
                               forwardConfig);
    m_drivetrain.AddTrajectory(kThirdStar, {}, kEndPose, backwardConfig);

    if (!m_autonChooser.Suspend([=] { return m_drivetrain.AtGoal(); })) {
        return;
    }
}
}  // namespace frc3512
