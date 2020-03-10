// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/geometry/Pose2d.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

enum class State { kInit, kIdle };
static State state;

void Robot::AutoLoadingZoneDriveForwardInit() { state = State::kInit; }

void Robot::AutoLoadingZoneDriveForwardPeriodic() {
    switch (state) {
        case State::kInit: {
            // Initial Pose - X: 12.65 m  Y: 5.800  Heading: 1*pi rad
            m_drivetrain.Reset(
                frc::Pose2d(12.65_m, 5.800_m, units::radian_t{wpi::math::pi}));
            m_drivetrain.SetWaypoints(
                frc::Pose2d(12.65_m, 5.800_m, units::radian_t{wpi::math::pi}),
                {},
                frc::Pose2d(12.65_m - DrivetrainController::kLength, 5.800_m,
                            units::radian_t{wpi::math::pi}));
            state = State::kIdle;
            break;
        }
        case State::kIdle: {
            break;
        }
    }
}

}  // namespace frc3512