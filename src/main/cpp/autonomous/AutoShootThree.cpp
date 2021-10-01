// Copyright (c) FRC Team 3512. All Rights Reserved.

#include <wpi/numbers>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoShootThree() {
    // Initial Pose - Right in line with the three balls in the Trench Run
    const frc::Pose2d kInitialPose{12_m, 1.05_m,
                                   units::radian_t{wpi::numbers::pi}};
    // Drive forward slightly
    const frc::Pose2d kShootingPose{12_m - Drivetrain::kLength, 1.05_m,
                                    units::radian_t{wpi::numbers::pi}};

    const frc::Pose2d kEndPose{12_m + 2 * Drivetrain::kLength, 1.05_m,
                               units::radian_t{wpi::numbers::pi}};

    drivetrain.Reset(kInitialPose);
    drivetrain.AddTrajectory(kInitialPose, {}, kShootingPose);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    if constexpr (IsSimulation()) {
        for (int i = 0; i < 3; ++i) {
            intakeSim.AddBall();
        }
    }

    // Shoot our 3 preloaded balls
    ShootWithVision(3);

    if (!m_autonChooser.Suspend([=] { return !IsShooting(); })) {
        return;
    }

    {
        auto config = Drivetrain::MakeTrajectoryConfig();
        config.SetReversed(true);
        drivetrain.AddTrajectory({kShootingPose, kEndPose}, config);
    }

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    intake.Stop();

    EXPECT_TRUE(turret.AtGoal());
}

}  // namespace frc3512
