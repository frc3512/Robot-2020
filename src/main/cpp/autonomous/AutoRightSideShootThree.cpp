// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

namespace {
enum class State { kInit, kDriveAwayFromGoal, kIdle };
}  // namespace

static State state;
static frc2::Timer autonTimer;

void Robot::AutoRightSideShootThreeInit() {
    state = State::kInit;
    autonTimer.Reset();
    autonTimer.Start();
}

void Robot::AutoRightSideShootThreePeriodic() {
    switch (state) {
        case State::kInit: {
            // Initial Pose - X: 12.65 m  Y: 0.7500 m  Heading: 0 rad
            m_drivetrain.Reset(
                frc::Pose2d(12.65_m, 0.7500_m, units::radian_t{wpi::math::pi}));
            m_drivetrain.SetWaypoints(
                frc::Pose2d(12.65_m, 0.7500_m, units::radian_t{wpi::math::pi}),
                {},
                frc::Pose2d(12.65_m - Drivetrain::kLength, 0.7500_m,
                            units::radian_t{wpi::math::pi}));
            state = State::kDriveAwayFromGoal;
            break;
        }
        case State::kDriveAwayFromGoal: {
            // Shoot x3
            if (m_drivetrain.AtGoal()) {
                Shoot();
                state = State::kIdle;
            }
            break;
        }
        case State::kIdle: {
            // Final Pose -
            // X: 12.65 m - RobotLength  Y: 0.7500 m  Heading: 0 rad
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
