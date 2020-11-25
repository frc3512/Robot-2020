// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <wpi/math>
#include <wpi/raw_ostream.h>

#include "Robot.hpp"

namespace frc3512 {

namespace {
enum class State { kInit, kShoot, kIdle };
}  // namespace

static State state;
static frc2::Timer autonTimer;

// Initial Pose - Right in line with the three balls in the Trench Run
static const frc::Pose2d initialPose{12.89_m, 0.71_m,
                                     units::radian_t{wpi::math::pi}};
// Drive forward slightly
static const frc::Pose2d endPose{12.89_m - 1.5 * Drivetrain::kLength, 0.71_m,
                                 units::radian_t{wpi::math::pi}};

void Robot::AutoRightSideShootThreeInit() {
    wpi::outs() << "RightSideShootThree autonomous\n";

    m_drivetrain.Reset(initialPose);

    state = State::kInit;
    autonTimer.Reset();
    autonTimer.Start();
}

void Robot::AutoRightSideShootThreePeriodic() {
    switch (state) {
        case State::kInit: {
            m_drivetrain.SetWaypoints(initialPose, {}, endPose);
            state = State::kShoot;
            break;
        }
        case State::kShoot: {
            if (m_drivetrain.AtGoal()) {
                // Shoot x3
                Shoot();
                state = State::kIdle;
            }
            break;
        }
        case State::kIdle: {
            break;
        }
    }

    if constexpr (IsSimulation()) {
        if (autonTimer.HasElapsed(14.97_s)) {
            EXPECT_EQ(State::kIdle, state);
        }
    }
}

}  // namespace frc3512
