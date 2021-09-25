// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include <frc/geometry/Pose2d.h>
#include <wpi/numbers>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoRightSideDriveForward() {
    // Initial Pose - Right in line with the three balls in the Trench Run
    const frc::Pose2d kInitialPose{12_m, 1_m,
                                   units::radian_t{wpi::numbers::pi}};
    // End Pose - Drive forward slightly
    const frc::Pose2d kEndPose{12_m - 1.5 * Drivetrain::kLength, 1_m,
                               units::radian_t{wpi::numbers::pi}};

    drivetrain.Reset(kInitialPose);
    drivetrain.AddTrajectory(kInitialPose, {}, kEndPose);

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtGoal(); })) {
        return;
    }

    EXPECT_TRUE(turret.AtGoal());
}

}  // namespace frc3512
