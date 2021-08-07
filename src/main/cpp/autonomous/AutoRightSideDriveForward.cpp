// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include <frc/geometry/Pose2d.h>
#include <wpi/numbers>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoRightSideDriveForward() {
    // Initial Pose - Right in line with the three balls in the Trench Run
    const frc::Pose2d kInitialPose{12.89_m, 0.71_m,
                                   units::radian_t{wpi::numbers::pi}};
    // End Pose - Drive forward slightly
    const frc::Pose2d kEndPose{12.89_m - 1.5 * Drivetrain::kLength, 0.71_m,
                               units::radian_t{wpi::numbers::pi}};

    m_drivetrain.Reset(kInitialPose);
    m_drivetrain.AddTrajectory(kInitialPose, {}, kEndPose);

    if (!m_autonChooser.Suspend([=] { return m_drivetrain.AtGoal(); })) {
        return;
    }
}

}  // namespace frc3512
