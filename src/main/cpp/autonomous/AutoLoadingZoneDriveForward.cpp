// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include <frc/geometry/Pose2d.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoLoadingZoneDriveForward() {
    // Initial Pose - Right in line with the Loading Zone
    const frc::Pose2d kInitialPose{12.89_m, 5.662_m,
                                   units::radian_t{wpi::math::pi}};
    // End Pose - Drive forward slightly
    const frc::Pose2d kEndPose{12.89_m - 1.5 * Drivetrain::kLength, 5.662_m,
                               units::radian_t{wpi::math::pi}};

    drivetrain.Reset(kInitialPose);
    drivetrain.AddTrajectory(kInitialPose, {}, kEndPose);

    while (!drivetrain.AtGoal()) {
        if (!m_autonChooser.Suspend()) {
            return;
        }
    }
}

}  // namespace frc3512
