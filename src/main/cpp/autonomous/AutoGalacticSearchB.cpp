// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <wpi/numbers>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoGalacticSearchB() {
    const frc::Pose2d kInitialPose{0.347_m + 0.762_m, 2.801_m,
                                   units::radian_t{0}};

    const frc::Pose2d kFirstRedBall{2.38_m - 0.3_m, 2.801_m,
                                    units::radian_t{0}};
    const frc::Pose2d kSecondRedBall{3.553_m, 1.529_m, units::radian_t{0}};
    const frc::Pose2d kThirdRedBall{
        5.38_m, 3.42_m, units::radian_t{13 * wpi::numbers::pi / 36}};
    const frc::Pose2d kThirdtoEndRed{6.635_m, 3.507_m, units::radian_t{0}};
    const frc::Pose2d kRedEndPose{8.532_m, 3.507_m, units::radian_t{0}};

    const frc::Pose2d kFirstBlueBall{4.54_m, 1.54_m, units::radian_t{0}};
    const frc::Pose2d kFirstToSecondBlueBall{
        5.351_m, 2.258_m, units::radian_t{wpi::numbers::pi / 4}};
    const frc::Pose2d kSecondBlueBall{6.017_m, 2.999_m,
                                      units::radian_t{wpi::numbers::pi / 4}};
    const frc::Pose2d kThirdBlueBall{
        7.225_m, 1.634_m, units::radian_t{35 * wpi::numbers::pi / 18}};
    const frc::Pose2d kBlueEndPose{8.591_m, 1.534_m, units::radian_t{0}};

    const units::meters_per_second_t kMaxV = 1_mps;

    m_turret.SetControlMode(TurretController::ControlMode::kManual);
    m_drivetrain.Reset(kInitialPose);

    m_intake.Deploy();
    m_intake.Start();

    // Wait for ultrasonic
    if (!m_autonChooser.SuspendFor(1.5_s)) {
        return;
    }

    if (m_drivetrain.GetUltrasonicDistance() <=
        ((kFirstRedBall.X() - kInitialPose.X()) + 0.3_m)) {
        m_autoGalacticSearchPath.SetString("Red");
        frc::RectangularRegionConstraint firstBallConstraint{
            frc::Translation2d{kFirstRedBall.X() - 0.5 * Drivetrain::kLength,
                               kFirstRedBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kFirstRedBall.X() + 0.5 * Drivetrain::kLength,
                               kFirstRedBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{kMaxV}};

        frc::RectangularRegionConstraint secondBallConstraint{
            frc::Translation2d{kSecondRedBall.X() - 0.5 * Drivetrain::kLength,
                               kSecondRedBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kSecondRedBall.X() + 0.5 * Drivetrain::kLength,
                               kSecondRedBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{kMaxV}};

        frc::RectangularRegionConstraint thirdBallConstraint{
            frc::Translation2d{kThirdRedBall.X() - 0.5 * Drivetrain::kLength,
                               kThirdRedBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kThirdRedBall.X() + 0.5 * Drivetrain::kLength,
                               kThirdRedBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{kMaxV}};

        auto config = Drivetrain::MakeTrajectoryConfig();
        config.AddConstraint(firstBallConstraint);
        config.AddConstraint(secondBallConstraint);
        config.AddConstraint(thirdBallConstraint);
        m_drivetrain.AddTrajectory({kInitialPose, kFirstRedBall, kSecondRedBall,
                                    kThirdRedBall, kThirdtoEndRed, kRedEndPose},
                                   config);
        if (!m_autonChooser.Suspend([=] { return m_drivetrain.AtGoal(); })) {
            return;
        }
        m_intake.Stop();
    } else {
        m_autoGalacticSearchPath.SetString("Blue");
        frc::RectangularRegionConstraint firstBallConstraint{
            frc::Translation2d{kFirstBlueBall.X() - 0.5 * Drivetrain::kLength,
                               kFirstBlueBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kFirstBlueBall.X() + 0.5 * Drivetrain::kLength,
                               kFirstBlueBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{kMaxV}};

        frc::RectangularRegionConstraint secondBallConstraint{
            frc::Translation2d{kSecondBlueBall.X() - 0.5 * Drivetrain::kLength,
                               kSecondBlueBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kSecondBlueBall.X() + 0.5 * Drivetrain::kLength,
                               kSecondBlueBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{kMaxV}};

        frc::RectangularRegionConstraint thirdBallConstraint{
            frc::Translation2d{kThirdBlueBall.X() - 0.5 * Drivetrain::kLength,
                               kThirdBlueBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kThirdBlueBall.X() + 0.5 * Drivetrain::kLength,
                               kThirdBlueBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{kMaxV}};

        auto config = Drivetrain::MakeTrajectoryConfig();
        config.AddConstraint(firstBallConstraint);
        config.AddConstraint(secondBallConstraint);
        config.AddConstraint(thirdBallConstraint);
        m_drivetrain.AddTrajectory(
            {kInitialPose, kFirstBlueBall, kFirstToSecondBlueBall,
             kSecondBlueBall, kThirdBlueBall, kBlueEndPose},
            config);
        if (!m_autonChooser.Suspend([=] { return m_drivetrain.AtGoal(); })) {
            return;
        }
        m_intake.Stop();
    }
}
}  // namespace frc3512
