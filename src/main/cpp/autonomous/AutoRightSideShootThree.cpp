// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoRightSideShootThree() {
    // Initial Pose - Right in line with the three balls in the Trench Run
    const frc::Pose2d kInitialPose{12.89_m, 0.71_m,
                                   units::radian_t{wpi::math::pi}};
    // Drive forward slightly
    const frc::Pose2d kEndPose{12.89_m - 1.5 * Drivetrain::kLength, 0.71_m,
                               units::radian_t{wpi::math::pi}};

    drivetrain.Reset(kInitialPose);
    drivetrain.AddTrajectory(kInitialPose, {}, kEndPose);

    while (!drivetrain.AtGoal()) {
        if (!m_autonChooser.Suspend()) {
            return;
        }
    }

    if constexpr (IsSimulation()) {
        for (int i = 0; i < 3; ++i) {
            intakeSim.AddBall();
        }
    }
    Shoot(3);

    while (IsShooting()) {
        if (!m_autonChooser.Suspend()) {
            return;
        }
    }
}

}  // namespace frc3512
