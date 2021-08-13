// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include <frc/geometry/Pose2d.h>
#include <wpi/numbers>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoTargetZoneShootThree() {
    const frc::Pose2d kInitialPose{12.89_m, 2.41_m,
                                   units::radian_t{wpi::numbers::pi}};
    const frc::Pose2d kEndPose{12.89_m - 1.5 * Drivetrain::kLength, 2.41_m,
                               units::radian_t{wpi::numbers::pi}};

    drivetrain.Reset(kInitialPose);
    drivetrain.AddTrajectory(kInitialPose, {}, kEndPose);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    if constexpr (IsSimulation()) {
        for (int i = 0; i < 3; ++i) {
            intakeSim.AddBall();
        }
    }
    Shoot(3);

    if (!m_autonChooser.Suspend([=] { return !IsShooting(); })) {
        return;
    }
}

}  // namespace frc3512
