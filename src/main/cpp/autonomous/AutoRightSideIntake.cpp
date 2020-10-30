// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

namespace {
enum class State { kInit, kTrenchRun, kIdle };
}  // namespace

static State state;
static frc2::Timer autonTimer;

void Robot::AutoRightSideIntakeInit() {
    m_drivetrain.Reset(
        frc::Pose2d(12.65_m, 0.7500_m, units::radian_t{wpi::math::pi}));

    state = State::kInit;
    autonTimer.Reset();
    autonTimer.Start();
}

void Robot::AutoRightSideIntakePeriodic() {
    switch (state) {
        case State::kInit: {
            m_drivetrain.SetWaypoints(
                frc::Pose2d(12.65_m, 0.7500_m, units::radian_t{wpi::math::pi}),
                {frc::Translation2d(10.50_m, 0.7500_m)},
                frc::Pose2d(7.775_m, 0.7500_m, units::radian_t{wpi::math::pi}));
            state = State::kTrenchRun;
            break;
        }
        case State::kTrenchRun: {
            // Intake Balls x3
            // TODO: Change if-statement condition to suit constraint
            if (1) {
                m_intake.SetArmMotor(Intake::ArmMotorDirection::kIntake);
                m_intake.SetFunnel(0.4);
                state = State::kIdle;
            }
            break;
        }
        case State::kIdle: {
            // Stop Intake
            // Final Pose -
            // X: 7.775 m  Y: 0.7500 m  Heading: 1*pi rad
            if (m_drivetrain.AtGoal()) {
                m_intake.SetArmMotor(Intake::ArmMotorDirection::kIdle);
                m_intake.SetFunnel(0);
            }
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
