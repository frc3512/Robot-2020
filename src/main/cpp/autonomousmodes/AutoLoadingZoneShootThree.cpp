// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/geometry/Pose2d.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

enum class State { kInit, kDriveAwayFromGoal, kIdle };
static State state;

void Robot::AutoLoadingZoneShootThreeInit() { state = State::kInit; }

void Robot::AutoLoadingZoneShootThreePeriodic() {
    switch (state) {
        case State::kInit: {
            // Initial Pose - X: 12.65 m  Y: 5.800 m  Heading: 0 rad
            m_drivetrain.Reset(
                frc::Pose2d(12.65_m, 5.800_m, units::radian_t{wpi::math::pi}));
            m_drivetrain.SetWaypoints(
                frc::Pose2d(12.65_m, 5.800_m, units::radian_t{wpi::math::pi}),
                {},
                frc::Pose2d(12.65_m - DrivetrainController::kLength, 5.800_m,
                            units::radian_t{wpi::math::pi}));
            state = State::kDriveAwayFromGoal;
            break;
        }
        case State::kDriveAwayFromGoal: {
            // Shoot x3
            if (m_drivetrain.AtGoal()) {
                m_flywheel.Shoot();
                state = State::kIdle;
            }
            break;
        }
        case State::kIdle: {
            break;
        }
    }
}

}  // namespace frc3512
