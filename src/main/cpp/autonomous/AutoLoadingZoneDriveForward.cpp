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

// Initial Pose - Right in line with the Loading Zone
static const frc::Pose2d initialPose{12.89_m, 5.662_m,
                                     units::radian_t{wpi::math::pi}};
// End Pose - Drive forward slightly
static const frc::Pose2d endPose{12.89_m - 1.5 * Drivetrain::kLength, 5.662_m,
                                 units::radian_t{wpi::math::pi}};

void Robot::AutoLoadingZoneDriveForwardInit() {
    wpi::outs() << "LoadingZoneDriveForward autonomous\n";

    m_drivetrain.Reset(initialPose);

    state = State::kInit;
    autonTimer.Reset();
    autonTimer.Start();
}

void Robot::AutoLoadingZoneDriveForwardPeriodic() {
    switch (state) {
        case State::kInit: {
            m_drivetrain.SetWaypoints(initialPose, {}, endPose);
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
            EXPECT_EQ(m_flywheel.GetGoal(), 0_rad_per_s);
            EXPECT_TRUE(m_turret.AtGoal());
        }
    }
}

}  // namespace frc3512
