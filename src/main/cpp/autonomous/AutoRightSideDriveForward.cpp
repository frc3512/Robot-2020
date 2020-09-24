// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/geometry/Pose2d.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

namespace {
enum class State { kInit, kIdle };
}  // namespace

static State state;

void Robot::AutoRightSideDriveForwardInit() { state = State::kInit; }

void Robot::AutoRightSideDriveForwardPeriodic() {
    switch (state) {
        case State::kInit: {
            // Initial Pose - X: 12.65 m  Y: 5.800  Heading: 1*pi rad
            m_drivetrain.Reset(frc::Pose2d(12.65_m, 0.7500_m + kPathWeaverFudge,
                                           units::radian_t{wpi::math::pi}));
            m_drivetrain.SetWaypoints(
                frc::Pose2d(12.65_m, 0.7500_m + kPathWeaverFudge,
                            units::radian_t{wpi::math::pi}),
                {},
                frc::Pose2d(12.65_m - Drivetrain::kLength,
                            0.7500_m + kPathWeaverFudge,
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
