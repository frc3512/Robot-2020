// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/geometry/Pose2d.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

namespace {
enum class State { kShoot, kDriveAwayFromGoal, kIdle };
}  // namespace

static State state;

void Robot::AutoLoadingZoneShootThreeInit() { state = State::kShoot; }

void Robot::AutoLoadingZoneShootThreePeriodic() {
    switch (state) {
        case State::kShoot: {
            // Shoot x3
            m_drivetrain.Reset(frc::Pose2d(12.65_m, 5.800_m + kPathWeaverFudge,
                                           units::radian_t{wpi::math::pi}));
            m_vision.TurnLEDOn();
            m_flywheel.Shoot();
            m_timer.Start();
            state = State::kDriveAwayFromGoal;
            break;
        }
        case State::kDriveAwayFromGoal: {
            if (m_timer.HasElapsed(3.0_s)) {
                m_timer.Stop();
                m_timer.Reset();
                m_flywheel.SetGoal(0.0_rad_per_s);
                m_vision.TurnLEDOff();
                // Initial Pose - X: 12.65 m  Y: 0.7500 m  Heading: 0 rad
                m_drivetrain.SetWaypoints(
                    frc::Pose2d(12.65_m, 5.800_m + kPathWeaverFudge,
                                units::radian_t{wpi::math::pi}),
                    {},
                    frc::Pose2d(12.65_m - Drivetrain::kLength - 0.5_m,
                                5.800_m + kPathWeaverFudge,
                                units::radian_t{wpi::math::pi}));
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
