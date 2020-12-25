// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/geometry/Pose2d.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoRightSideDriveForward() {
    // Initial Pose - Right in line with the three balls in the Trench Run
    const frc::Pose2d initialPose{12.89_m, 0.71_m,
                                  units::radian_t{wpi::math::pi}};
    // End Pose - Drive forward slightly
    const frc::Pose2d endPose{12.89_m - 1.5 * Drivetrain::kLength, 0.71_m,
                              units::radian_t{wpi::math::pi}};

    m_drivetrain.Reset(initialPose);
    m_drivetrain.SetWaypoints(initialPose, {}, endPose);

    while (!m_drivetrain.AtGoal()) {
        m_autonChooser.YieldToMain();
        if (!IsAutonomousEnabled()) {
            EXPECT_TRUE(false) << "Autonomous mode didn't complete";
            return;
        }
    }
}

}  // namespace frc3512
