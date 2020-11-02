// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

namespace {
enum class State {
    kInit,
    kShoot,
    kTrenchRun,
    kTrenchIntake,
    kTrenchShoot,
    kIdle
};
}  // namespace

// TODO: This autonomous mode doesn't pass unit tests

static State state;
static frc2::Timer autonTimer;

static const frc::Pose2d initialPose{12.89_m, 2.41_m,
                                     units::radian_t{wpi::math::pi}};
static const frc::Pose2d midPose{12.89_m - 1.5 * Drivetrain::kLength, 2.41_m,
                                 units::radian_t{wpi::math::pi}};
static const frc::Pose2d endPose{8_m, 0.75_m, units::radian_t{wpi::math::pi}};

void Robot::AutoTargetZoneShootSixInit() {
    wpi::outs() << "TargetZoneShootSix autonomous\n";

    m_drivetrain.Reset(initialPose);

    state = State::kInit;
    autonTimer.Reset();
    autonTimer.Start();
}

void Robot::AutoTargetZoneShootSixPeriodic() {
    switch (state) {
        case State::kInit: {
            // Inital Pose: X: 12.91 m Y: 2.6 m Heading: pi rad
            // Back up to shoot three
            m_drivetrain.SetWaypoints(initialPose, {}, midPose);
            state = State::kShoot;
            break;
        }
        case State::kShoot: {
            if (m_drivetrain.AtGoal()) {
                // Shoot x3
                Shoot();
                state = State::kTrenchRun;
            }
            break;
        }
        case State::kTrenchRun: {
            // Intake Balls x3
            // Middle Pose - X: 10.5 m Y: 0.75 m Heading: pi rad
            // TODO: Add translation to make sure all balls are picked up.
            if (!IsShooting()) {
                m_drivetrain.SetWaypoints(midPose, {}, endPose);
                state = State::kTrenchShoot;
            }
            break;
        }
        case State::kTrenchIntake: {
            // Intake Balls x3
            // TODO: Add velocity constraint
            m_intake.SetArmMotor(Intake::ArmMotorDirection::kIntake);
            m_intake.SetConveyor(0.85);
            state = State::kTrenchShoot;
            break;
        }
        case State::kTrenchShoot: {
            // Final Pose - X: 8 m Y: 0.75 m Heading: pi rad
            if (m_drivetrain.AtGoal()) {
                m_intake.SetArmMotor(Intake::ArmMotorDirection::kIdle);
                m_intake.SetConveyor(0.0);
                // Shoot x3
                Shoot();
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
