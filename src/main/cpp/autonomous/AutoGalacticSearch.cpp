// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <wpi/numbers>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoGalacticSearch() {
    // Start waypoint
    const frc::Pose2d kInitialPose{0.514_m + 0.762_m, 2.616_m,
                                   units::radian_t{0}};

    // Waypoints for Path A layout

    const frc::Pose2d kAFirstRedBall{2.367_m, 2.294_m, units::radian_t{0}};
    const frc::Pose2d kASecondRedBall{3.875_m - 0.25_m, 1.528_m + 0.5_m,
                                      units::radian_t{0}};
    const frc::Pose2d kAThirdRedBall{4.642_m, 3.892_m, units::radian_t{0}};

    const frc::Pose2d kARedEndPose{8.618_m, 3.892_m, units::radian_t{0}};

    const frc::Pose2d kAFirstBlueBall{4.642_m - 0.25_m, 0.837_m + 0.25_m,
                                      units::radian_t{0}};
    const frc::Pose2d kASecondBlueBall{5.409_m, 3.049_m + 0.25_m,
                                       units::radian_t{0}};
    const frc::Pose2d kAThirdBlueBall{
        6.969_m, 2.282_m + 0.45_m,
        units::radian_t{((7 * wpi::numbers::pi) / 4)}};

    const frc::Pose2d kABlueEndPose{
        8.618_m, 1.083_m, units::radian_t{((7 * wpi::numbers::pi) / 4)}};

    // Waypoints for Path B layout
    const frc::Pose2d kBFirstRedBall{2.38_m - 0.3_m, 2.801_m,
                                     units::radian_t{0}};
    const frc::Pose2d kBSecondRedBall{3.553_m, 1.529_m, units::radian_t{0}};
    const frc::Pose2d kBThirdRedBall{
        5.38_m, 3.42_m, units::radian_t{13 * wpi::numbers::pi / 36}};
    const frc::Pose2d kBThirdtoEndRed{6.635_m, 3.507_m, units::radian_t{0}};
    const frc::Pose2d kBRedEndPose{8.532_m, 3.507_m, units::radian_t{0}};

    const frc::Pose2d kBFirstBlueBall{4.54_m, 1.54_m, units::radian_t{0}};
    const frc::Pose2d kBFirstToSecondBlueBall{
        5.351_m, 2.258_m, units::radian_t{wpi::numbers::pi / 4}};
    const frc::Pose2d kBSecondBlueBall{6.017_m, 2.999_m,
                                       units::radian_t{wpi::numbers::pi / 4}};
    const frc::Pose2d kBThirdBlueBall{
        7.225_m, 1.634_m, units::radian_t{35 * wpi::numbers::pi / 18}};

    const frc::Pose2d kBBlueEndPose{8.591_m, 1.534_m, units::radian_t{0}};

    // Max velocity for this auton mode to run in
    const units::meters_per_second_t kMaxV = 1_mps;

    turret.SetControlMode(TurretController::ControlMode::kManual);
    drivetrain.Reset(kInitialPose);

    intake.Deploy();
    intake.Start();
    intake.SetConveyor(0.7);
    if (!m_autonChooser.SuspendFor(1_s)) {
        return;
    }

    if (drivetrain.GetLeftUltrasonicDistance() <=
        (kBFirstRedBall.X() - kInitialPose.X())) {
        m_autoGalacticSearchLayout.SetString("Path B");
        m_autoGalacticSearchPath.SetString("Red");
        frc::RectangularRegionConstraint firstBallConstraint{
            frc::Translation2d{kBFirstRedBall.X() - 0.5 * Drivetrain::kLength,
                               kBFirstRedBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kBFirstRedBall.X() + 0.5 * Drivetrain::kLength,
                               kBFirstRedBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{kMaxV}};

        frc::RectangularRegionConstraint secondBallConstraint{
            frc::Translation2d{kBSecondRedBall.X() - 0.5 * Drivetrain::kLength,
                               kBSecondRedBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kBSecondRedBall.X() + 0.5 * Drivetrain::kLength,
                               kBSecondRedBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{kMaxV}};

        frc::RectangularRegionConstraint thirdBallConstraint{
            frc::Translation2d{kBThirdRedBall.X() - 0.5 * Drivetrain::kLength,
                               kBThirdRedBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kBThirdRedBall.X() + 0.5 * Drivetrain::kLength,
                               kBThirdRedBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{kMaxV}};

        auto config = Drivetrain::MakeTrajectoryConfig();
        config.AddConstraint(firstBallConstraint);
        config.AddConstraint(secondBallConstraint);
        config.AddConstraint(thirdBallConstraint);
        drivetrain.AddTrajectory(
            {kInitialPose, kBFirstRedBall, kBSecondRedBall, kBThirdRedBall,
             kBThirdtoEndRed, kBRedEndPose},
            config);
        if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
            return;
        }
        intake.Stop();
    } else if (drivetrain.GetRightUltrasonicDistance() <=
               (kAFirstRedBall.X() - kInitialPose.X())) {
        m_autoGalacticSearchLayout.SetString("Path A");
        m_autoGalacticSearchPath.SetString("Red");
        frc::RectangularRegionConstraint firstBallConstraint{
            frc::Translation2d{kAFirstRedBall.X() - 0.5 * Drivetrain::kLength,
                               kAFirstRedBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kAFirstRedBall.X() + 0.5 * Drivetrain::kLength,
                               kAFirstRedBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{kMaxV}};

        frc::RectangularRegionConstraint secondBallConstraint{
            frc::Translation2d{kASecondRedBall.X() - 0.5 * Drivetrain::kLength,
                               kASecondRedBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kASecondRedBall.X() + 0.5 * Drivetrain::kLength,
                               kASecondRedBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{kMaxV}};

        frc::RectangularRegionConstraint thirdBallConstraint{
            frc::Translation2d{kAThirdRedBall.X() - 0.5 * Drivetrain::kLength,
                               kAThirdRedBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kAThirdRedBall.X() + 0.5 * Drivetrain::kLength,
                               kAThirdRedBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{kMaxV}};

        auto config = Drivetrain::MakeTrajectoryConfig();
        config.AddConstraint(firstBallConstraint);
        config.AddConstraint(secondBallConstraint);
        config.AddConstraint(thirdBallConstraint);
        drivetrain.AddTrajectory({kInitialPose, kAFirstRedBall, kASecondRedBall,
                                  kAThirdRedBall, kARedEndPose},
                                 config);

        if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
            return;
        }
        intake.Stop();
    } else {
        m_autoGalacticSearchPath.SetString("Blue");

        frc::Pose2d seeBlue{0.514_m + 0.762_m + 105_in, 2.616_m,
                            units::radian_t{0}};
        drivetrain.AddTrajectory(kInitialPose, {}, seeBlue);

        if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
            return;
        }

        if (!m_autonChooser.SuspendFor(1_s)) {
            return;
        }

        if (drivetrain.GetLeftUltrasonicDistance() <=
            (kBFirstRedBall.X() - kInitialPose.X())) {
            m_autoGalacticSearchLayout.SetString("Path B");
            frc::RectangularRegionConstraint firstBallConstraint{
                frc::Translation2d{
                    kBFirstBlueBall.X() - 0.5 * Drivetrain::kLength,
                    kBFirstBlueBall.Y() - 0.5 * Drivetrain::kLength},
                frc::Translation2d{
                    kBFirstBlueBall.X() + 0.5 * Drivetrain::kLength,
                    kBFirstBlueBall.Y() + 0.5 * Drivetrain::kLength},
                frc::MaxVelocityConstraint{kMaxV}};

            frc::RectangularRegionConstraint secondBallConstraint{
                frc::Translation2d{
                    kBSecondBlueBall.X() - 0.5 * Drivetrain::kLength,
                    kBSecondBlueBall.Y() - 0.5 * Drivetrain::kLength},
                frc::Translation2d{
                    kBSecondBlueBall.X() + 0.5 * Drivetrain::kLength,
                    kBSecondBlueBall.Y() + 0.5 * Drivetrain::kLength},
                frc::MaxVelocityConstraint{kMaxV}};

            frc::RectangularRegionConstraint thirdBallConstraint{
                frc::Translation2d{
                    kBThirdBlueBall.X() - 0.5 * Drivetrain::kLength,
                    kBThirdBlueBall.Y() - 0.5 * Drivetrain::kLength},
                frc::Translation2d{
                    kBThirdBlueBall.X() + 0.5 * Drivetrain::kLength,
                    kBThirdBlueBall.Y() + 0.5 * Drivetrain::kLength},
                frc::MaxVelocityConstraint{kMaxV}};

            auto config = Drivetrain::MakeTrajectoryConfig();
            config.AddConstraint(firstBallConstraint);
            config.AddConstraint(secondBallConstraint);
            config.AddConstraint(thirdBallConstraint);
            drivetrain.AddTrajectory(
                {seeBlue, kBFirstBlueBall, kBFirstToSecondBlueBall,
                 kBSecondBlueBall, kBThirdBlueBall, kBBlueEndPose},
                config);
            if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
                return;
            }
            intake.Stop();
        } else {
            m_autoGalacticSearchLayout.SetString("Path A");
            frc::RectangularRegionConstraint firstBallConstraint{
                frc::Translation2d{
                    kAFirstBlueBall.X() - 0.5 * Drivetrain::kLength,
                    kAFirstBlueBall.Y() - 0.5 * Drivetrain::kLength},
                frc::Translation2d{
                    kAFirstBlueBall.X() + 0.5 * Drivetrain::kLength,
                    kAFirstBlueBall.Y() + 0.5 * Drivetrain::kLength},
                frc::MaxVelocityConstraint{kMaxV}};

            frc::RectangularRegionConstraint secondBallConstraint{
                frc::Translation2d{
                    kASecondBlueBall.X() - 0.5 * Drivetrain::kLength,
                    kASecondBlueBall.Y() - 0.5 * Drivetrain::kLength},
                frc::Translation2d{
                    kASecondBlueBall.X() + 0.5 * Drivetrain::kLength,
                    kASecondBlueBall.Y() + 0.5 * Drivetrain::kLength},
                frc::MaxVelocityConstraint{kMaxV}};

            frc::RectangularRegionConstraint thirdBallConstraint{
                frc::Translation2d{
                    kAThirdBlueBall.X() - 0.5 * Drivetrain::kLength,
                    kAThirdBlueBall.Y() - 0.5 * Drivetrain::kLength},
                frc::Translation2d{
                    kAThirdBlueBall.X() + 0.5 * Drivetrain::kLength,
                    kAThirdBlueBall.Y() + 0.5 * Drivetrain::kLength},
                frc::MaxVelocityConstraint{kMaxV}};

            auto config = Drivetrain::MakeTrajectoryConfig();
            config.AddConstraint(firstBallConstraint);
            config.AddConstraint(secondBallConstraint);
            config.AddConstraint(thirdBallConstraint);
            drivetrain.AddTrajectory(
                {seeBlue, kAFirstBlueBall, kASecondBlueBall, kAThirdBlueBall,
                 kABlueEndPose},
                config);

            if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
                return;
            }
            intake.Stop();
        }
    }
}
}  // namespace frc3512
