// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

namespace {
enum class State { kInit, kShoot, kTrenchRun, kTrenchShoot, kIdle };
}  // namespace

static State state;
static frc2::Timer autonTimer;

void Robot::AutoTargetZoneShootSixInit() {
    state = State::kInit;
    autonTimer.Reset();
    autonTimer.Start();
}

void Robot::AutoTargetZoneShootSixPeriodic() {
    switch (state) {
        case State::kInit: {
            // Initial Pose - X: 12.65 m  Y: 2.600 m  Heading: pi rad
            m_drivetrain.Reset(
                frc::Pose2d(12.65_m, 2.600_m, units::radian_t{wpi::math::pi}));
            m_drivetrain.SetWaypoints(
                frc::Pose2d(12.65_m, 2.600_m, units::radian_t{wpi::math::pi}),
                {},
                frc::Pose2d(10.50_m, 0.7500_m, units::radian_t{wpi::math::pi}));
            state = State::kShoot;
            break;
        }
        case State::kShoot: {
            // Shoot x3
            // Change if-statement condition to suit constraint
            if (m_drivetrain.AtGoal()) {
                m_flywheel.Shoot();
                state = State::kTrenchRun;
            }
            break;
        }
        case State::kTrenchRun: {
            // Intake Balls x3
            // Final Pose - X: 7.775 m + RobotLength  Y: 0.7500 m  Heading: 1*pi
            // Shoot x3
            if (m_timer.HasElapsed(3_s)) {
                m_intake.SetArmMotor(Intake::ArmMotorDirection::kIntake);
                state = State::kTrenchShoot;
            }
            break;
        }
        case State::kTrenchShoot: {
            break;
        }
        case State::kIdle: {
            if (m_timer.HasElapsed(10_s)) {
                m_timer.Stop();
                m_intake.SetArmMotor(Intake::ArmMotorDirection::kIdle);
                m_intake.SetConveyor(0.0);
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
