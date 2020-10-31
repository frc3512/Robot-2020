// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/geometry/Pose2d.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

namespace {
enum class State { kInit, kDriveAwayFromGoal, kIdle };
}  // namespace

static State state;
static frc2::Timer autonTimer;

void Robot::AutoTargetZoneShootThreeInit() {
    m_drivetrain.Reset(frc::Pose2d(12.65_m, 2.6_m + kPathWeaverFudge,
                                   units::radian_t{wpi::math::pi}));

    state = State::kInit;
    autonTimer.Reset();
    autonTimer.Start();
}

void Robot::AutoTargetZoneShootThreePeriodic() {
    switch (state) {
        case State::kInit: {
            m_drivetrain.SetWaypoints(
                frc::Pose2d(12.65_m, 2.6_m + kPathWeaverFudge,
                            units::radian_t{wpi::math::pi}),
                {},
                frc::Pose2d(12.65_m - Drivetrain::kLength,
                            2.6_m + kPathWeaverFudge,
                            units::radian_t{wpi::math::pi}));
            state = State::kDriveAwayFromGoal;
            break;
        }
        case State::kDriveAwayFromGoal: {
            if (m_drivetrain.AtGoal()) {
                m_vision.TurnLEDOn();
                m_flywheel.Shoot();
                m_timer.Start();
                state = State::kIdle;
            }
            break;
        }
        case State::kIdle: {
            if (m_timer.HasElapsed(3.0_s)) {
                m_flywheel.SetGoal(0.0_rad_per_s);
                m_vision.TurnLEDOff();
            }
            break;
        }
    }

    if constexpr (IsSimulation()) {
        if (autonTimer.HasElapsed(14.5_s)) {
            EXPECT_EQ(State::kIdle, state);
            EXPECT_TRUE(m_drivetrain.AtGoal());
            EXPECT_TRUE(m_flywheel.AtGoal());
        }
    }
}

}  // namespace frc3512
