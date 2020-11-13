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

static const frc::Pose2d initialPose{12.89_m, 2.41_m,
                                     units::radian_t{wpi::math::pi}};
static const frc::Pose2d endPose{12.89_m - 1.5 * Drivetrain::kLength, 2.41_m,
                                 units::radian_t{wpi::math::pi}};

void Robot::AutoTargetZoneShootThreeInit() {
    wpi::outs() << "TargetZoneShootThree autonomous\n";

    m_drivetrain.Reset(initialPose);

    state = State::kInit;
    autonTimer.Reset();
    autonTimer.Start();
}

void Robot::AutoTargetZoneShootThreePeriodic() {
    switch (state) {
        case State::kInit: {
            m_drivetrain.SetWaypoints(initialPose, {}, endPose);
            state = State::kDriveAwayFromGoal;
            break;
        }
        case State::kDriveAwayFromGoal: {
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
        if (autonTimer.HasElapsed(14.5_s)) {
            EXPECT_EQ(State::kIdle, state);
            EXPECT_TRUE(m_drivetrain.AtGoal());
            EXPECT_EQ(m_flywheel.GetGoal(), 0_rad_per_s);
            EXPECT_TRUE(m_turret.AtGoal());
        }
    }
}

}  // namespace frc3512
