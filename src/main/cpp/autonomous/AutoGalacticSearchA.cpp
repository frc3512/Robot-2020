// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <wpi/numbers>

#include "Robot.hpp"

namespace frc3512 {

// TODO: Mock ultrasonic as AnalogInputSim so both paths are exercised in unit
// testing
void Robot::AutoGalacticSearchA() {
    const frc::Pose2d kInitialPose{0.514_m + 0.762_m, 2.43_m,
                                   units::radian_t{0}};

    const frc::Pose2d kFirstRedBall{2.367_m, 2.294_m, units::radian_t{0}};
    const frc::Pose2d kSecondRedBall{3.875_m - 0.25_m, 1.528_m + 0.5_m,
                                     units::radian_t{0}};
    const frc::Pose2d kThirdRedBall{4.642_m, 3.892_m, units::radian_t{0}};

    const frc::Pose2d kFirstBlueBall{4.642_m - 0.25_m, 0.837_m + 0.25_m,
                                     units::radian_t{0}};
    const frc::Pose2d kSecondBlueBall{5.409_m, 3.049_m + 0.25_m,
                                      units::radian_t{0}};
    const frc::Pose2d kThirdBlueBall{
        6.969_m, 2.282_m + 0.45_m,
        units::radian_t{((7 * wpi::numbers::pi) / 4)}};

    const frc::Pose2d kBlueEndPose{
        8.618_m, 1.083_m, units::radian_t{((7 * wpi::numbers::pi) / 4)}};
    const frc::Pose2d kRedEndPose{8.618_m, 3.892_m, units::radian_t{0}};

    m_turret.SetControlMode(TurretController::ControlMode::kManual);
    m_drivetrain.Reset(kInitialPose);

    m_intake.Deploy();
    m_intake.Start();
    m_intake.SetConveyor(0.7);
    if (!m_autonChooser.SuspendFor(1_s)) {
        return;
    }

    // TODO: Make this tolerance wider
    if (m_drivetrain.GetUltrasonicDistance() <=
        (kFirstRedBall.X() - kInitialPose.X())) {
        m_autoGalacticSearchPath.SetString("Red");
        frc::RectangularRegionConstraint firstBallConstraint{
            frc::Translation2d{kFirstRedBall.X() - 0.5 * Drivetrain::kLength,
                               kFirstRedBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kFirstRedBall.X() + 0.5 * Drivetrain::kLength,
                               kFirstRedBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{1_mps}};

        frc::RectangularRegionConstraint secondBallConstraint{
            frc::Translation2d{kSecondRedBall.X() - 0.5 * Drivetrain::kLength,
                               kSecondRedBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kSecondRedBall.X() + 0.5 * Drivetrain::kLength,
                               kSecondRedBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{1_mps}};

        frc::RectangularRegionConstraint thirdBallConstraint{
            frc::Translation2d{kThirdRedBall.X() - 0.5 * Drivetrain::kLength,
                               kThirdRedBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kThirdRedBall.X() + 0.5 * Drivetrain::kLength,
                               kThirdRedBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{1_mps}};

        auto config = Drivetrain::MakeTrajectoryConfig();
        config.AddConstraint(firstBallConstraint);
        config.AddConstraint(secondBallConstraint);
        config.AddConstraint(thirdBallConstraint);
        m_drivetrain.AddTrajectory({kInitialPose, kFirstRedBall, kSecondRedBall,
                                    kThirdRedBall, kRedEndPose},
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
            frc::MaxVelocityConstraint{1_mps}};

        frc::RectangularRegionConstraint secondBallConstraint{
            frc::Translation2d{kSecondBlueBall.X() - 0.5 * Drivetrain::kLength,
                               kSecondBlueBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kSecondBlueBall.X() + 0.5 * Drivetrain::kLength,
                               kSecondBlueBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{1_mps}};

        frc::RectangularRegionConstraint thirdBallConstraint{
            frc::Translation2d{kThirdBlueBall.X() - 0.5 * Drivetrain::kLength,
                               kThirdBlueBall.Y() - 0.5 * Drivetrain::kLength},
            frc::Translation2d{kThirdBlueBall.X() + 0.5 * Drivetrain::kLength,
                               kThirdBlueBall.Y() + 0.5 * Drivetrain::kLength},
            frc::MaxVelocityConstraint{1_mps}};

        auto config = Drivetrain::MakeTrajectoryConfig();
        config.AddConstraint(firstBallConstraint);
        config.AddConstraint(secondBallConstraint);
        config.AddConstraint(thirdBallConstraint);
        m_drivetrain.AddTrajectory(
            {kInitialPose, kFirstBlueBall, kSecondBlueBall, kThirdBlueBall,
             kBlueEndPose},
            config);

        if (!m_autonChooser.Suspend([=] { return m_drivetrain.AtGoal(); })) {
            return;
        }
        m_intake.Stop();
    }
}
}  // namespace frc3512
