// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

namespace {
enum class State { kInit, kDoneShooting, kTrenchRun, kTrenchShoot, kIdle };
}  // namespace

// TODO: This autonomous mode doesn't pass unit tests

static State state;
static State lastState;
static frc2::Timer autonTimer;

// Initial Pose - Right in line with the Target Zone
static const frc::Pose2d initialPose{12.89_m, 2.41_m,
                                     units::radian_t{wpi::math::pi}};
// End Pose - Third/Farthest ball in the Trench Run
static const frc::Pose2d endPose{8_m, 0.71_m, units::radian_t{wpi::math::pi}};

void Robot::AutoTargetZoneShootSixInit() {
    wpi::outs() << "TargetZoneShootSix autonomous\n";

    m_drivetrain.Reset(initialPose);

    state = State::kInit;
    lastState = State::kInit;
    autonTimer.Reset();
    autonTimer.Start();
}

void Robot::AutoTargetZoneShootSixPeriodic() {
    switch (state) {
        case State::kInit: {
            // Shoot x3
            Shoot();
            state = State::kDoneShooting;
            break;
        }
        case State::kDoneShooting: {
            if (!IsShooting()) {
                state = State::kTrenchRun;
            }
            break;
        }
        case State::kTrenchRun: {
            static frc::RectangularRegionConstraint regionConstraint{
                // X: Leftmost ball on trench run
                frc::Translation2d{endPose.X(),
                                   0.71_m - 0.5 * Drivetrain::kLength},
                // X: Rightmost ball on trench run
                frc::Translation2d{9.82_m + 0.5 * Drivetrain::kLength,
                                   0.71_m + 0.5 * Drivetrain::kLength},
                frc::MaxVelocityConstraint{1_mps}};

            if (lastState != state) {
                // Add a constraint to slow down the drivetrain while it's
                // approaching the balls
                auto config = m_drivetrain.MakeTrajectoryConfig();

                config.AddConstraint(regionConstraint);
                // Interior Translation: First/Closest ball in trench run
                m_drivetrain.SetWaypoints(initialPose,
                                          {frc::Translation2d{9.82_m, 0.705_m}},
                                          endPose, config);
                m_intake.Deploy();

                lastState = state;
            }

            // Intake Balls x3
            if (regionConstraint.IsPoseInRegion(m_drivetrain.GetPose())) {
                m_intake.SetArmMotor(Intake::ArmMotorDirection::kIntake);
                m_intake.SetConveyor(0.85);
            }
            if (m_drivetrain.AtGoal()) {
                state = State::kTrenchShoot;
            }
            break;
        }
        case State::kTrenchShoot: {
            m_intake.SetArmMotor(Intake::ArmMotorDirection::kIdle);
            m_intake.SetConveyor(0.0);
            // Shoot x3
            Shoot();
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
