// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <wpi/math>

#include "Robot.hpp"
#include "trajectory/constraint/RectangularRegionConstraint.h"

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
static frc2::Timer autonTimer;

static const frc::Pose2d initialPose{12.89_m, 0.71_m,
                                     units::radian_t{wpi::math::pi}};
static const frc::Pose2d midPose{12.89_m - 1.5 * Drivetrain::kLength, 0.71_m,
                                 units::radian_t{wpi::math::pi}};
static const frc::Pose2d endPose{8_m, 0.71_m, units::radian_t{wpi::math::pi}};

void Robot::AutoRightSideShootSixInit() {
    wpi::outs() << "RightSideShootThree autonomous\n";

    m_drivetrain.Reset(initialPose);

    state = State::kInit;
    autonTimer.Reset();
    autonTimer.Start();
}

void Robot::AutoRightSideShootSixPeriodic() {
    switch (state) {
        case State::kInit: {
            // Move back to shoot three comfortably
            // Inital Pose - X: 12.91 m Y: 0.75 m Heading: pi rad
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
            // Middle Pose - X: 12.91 - klength - khalflength m Y: 0.75 m
            // Heading: pi rad
            if (!IsShooting()) {
                // Add a constraint to slow down the drivetrain while it's
                // approaching the balls
                frc::Translation2d bottomLeftConstraint{11_m, initialPose.Y()};
                frc::Translation2d topRightConstraint{endPose.X(), endPose.Y()};
                frc::MaxVelocityConstraint velocityConstraint{1.5_mps};

                // Make a trajectory config to add the rectangle constraint
                auto config = m_drivetrain.MakeTrajectoryConfig();
                config.AddConstraint(frc::RectangularRegionConstraint{
                    bottomLeftConstraint, topRightConstraint,
                    velocityConstraint});

                m_drivetrain.SetWaypoints(midPose, {}, endPose, config);

                state = State::kTrenchIntake;
            }
            break;
        }
        case State::kTrenchIntake: {
            // Intake Balls x3
            // TODO: Add if pose in region if statement
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
