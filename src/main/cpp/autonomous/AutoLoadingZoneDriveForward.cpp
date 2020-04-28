// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/geometry/Pose2d.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

namespace {
enum class State { kInit, kIdle };
}  // namespace

static State state;
static frc2::Timer autonTimer;

void Robot::AutoLoadingZoneDriveForwardInit() {
    state = State::kInit;
    autonTimer.Reset();
    autonTimer.Start();
}

void Robot::AutoLoadingZoneDriveForwardPeriodic() {
    switch (state) {
        case State::kInit: {
            frc::Pose2d initialPose{12.65_m, 5.800_m + kPathWeaverFudge,
                                    units::radian_t{wpi::math::pi}};
            m_drivetrain.SetWaypoints(
                initialPose, {},
                frc::Pose2d(12.65_m - Drivetrain::kLength - 0.5_m,
                            5.800_m + kPathWeaverFudge,
                            units::radian_t{wpi::math::pi}));
            m_drivetrain.Reset(initialPose);
            state = State::kIdle;
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
            EXPECT_TRUE(m_flywheel.AtGoal());
        }
    }
}

}  // namespace frc3512
