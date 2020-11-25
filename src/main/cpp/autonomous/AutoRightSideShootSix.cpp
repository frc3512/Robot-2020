// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

namespace {
enum class State {
    kInit,
    kShoot,
    kTrenchRun,
    kTrenchShoot,
    kTrenchIntake,
    kIdle
};
}  // namespace

static State state;
static State lastState;
static frc2::Timer autonTimer;

// Initial Pose - Right in line with the three balls in the Trench Run
static const frc::Pose2d initialPose{12.89_m, 0.71_m,
                                     units::radian_t{wpi::math::pi}};
// Mid Pose - Drive forward slightly
static const frc::Pose2d midPose{12.89_m - 1.5 * Drivetrain::kLength, 0.71_m,
                                 units::radian_t{wpi::math::pi}};
// End Pose - Third/Farthest ball in the Trench Run
static const frc::Pose2d endPose{8_m, 0.71_m, units::radian_t{wpi::math::pi}};

void Robot::AutoRightSideShootSixInit() {
    m_drivetrain.Reset(initialPose);

    state = State::kInit;
    lastState = State::kInit;
    autonTimer.Reset();
    autonTimer.Start();
}

void Robot::AutoRightSideShootSixPeriodic() {
    switch (state) {
        case State::kInit: {
            m_intake.Deploy();
            // Move back to shoot three comfortably
            m_drivetrain.SetWaypoints(initialPose, {}, midPose);
            state = State::kShoot;
            break;
        }
        case State::kShoot: {
            // Shoot x3
            if (m_drivetrain.AtGoal()) {
                Shoot();
                state = State::kTrenchRun;
            }
            break;
        }
        case State::kTrenchRun: {
            if (!IsShooting()) {
                state = State::kTrenchIntake;
            }
            break;
        }
        case State::kTrenchIntake: {
            // Add a constraint to slow down the drivetrain while it's
            // approaching the balls
            static frc::RectangularRegionConstraint regionConstraint{
                frc::Translation2d{endPose.X(),
                                   endPose.Y() - 0.5 * Drivetrain::kLength},
                // X: First/Closest ball in the trench run
                frc::Translation2d{9.82_m + 0.5 * Drivetrain::kLength,
                                   initialPose.Y() + 0.5 * Drivetrain::kLength},
                frc::MaxVelocityConstraint{1_mps}};

            if (lastState != state) {
                auto config = m_drivetrain.MakeTrajectoryConfig();
                config.AddConstraint(regionConstraint);
                m_drivetrain.SetWaypoints(midPose, {}, endPose, config);

                lastState = state;
            }

            // Intake Balls x3
            if (regionConstraint.IsPoseInRegion(m_drivetrain.GetPose())) {
                m_intake.SetArmMotor(Intake::ArmMotorDirection::kIntake);
                m_intake.SetFunnel(0.4);
            }

            if (m_drivetrain.AtGoal()) {
                state = State::kTrenchShoot;
            }
            break;
        }
        case State::kTrenchShoot: {
            m_intake.SetArmMotor(Intake::ArmMotorDirection::kIdle);
            m_intake.SetFunnel(0.0);
            // Shoot x3
            Shoot();
            state = State::kIdle;
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
