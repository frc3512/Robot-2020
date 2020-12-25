// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/geometry/Pose2d.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoTargetZoneShootThree() {
    const frc::Pose2d initialPose{12.89_m, 2.41_m,
                                  units::radian_t{wpi::math::pi}};
    const frc::Pose2d endPose{12.89_m - 1.5 * Drivetrain::kLength, 2.41_m,
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

    // Shoot x3
    Shoot();

    while (IsShooting()) {
        m_autonChooser.YieldToMain();
        if (!IsAutonomousEnabled()) {
            EXPECT_TRUE(false) << "Autonomous mode didn't complete";
            return;
        }
    }
}

}  // namespace frc3512
