// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

namespace {
enum class State { kInit, kDriving, kIdle };
}  // namespace

static State state;
static State lastState;
static frc2::Timer autonTimer;

// Inital Pose - On initiation line between two balls next to color wheel
static const frc::Pose2d initialPose{12.89_m, 7.513_m,
                                     units::radian_t{wpi::math::pi}};
// End Pose - Right before the two balls on the color wheel so intake doesn't
// hit it
static const frc::Pose2d endPose{9.63_m + Drivetrain::kMiddleOfRobotToIntake,
                                 7.513_m, units::radian_t{wpi::math::pi}};

void Robot::AutoLeftSideIntakeInit() {
    wpi::outs() << "LeftSideIntake autonomous\n";

    m_drivetrain.Reset(initialPose);

    state = State::kInit;
    lastState = State::kInit;
    autonTimer.Reset();
    autonTimer.Start();
}

void Robot::AutoLeftSideIntakePeriodic() {
    switch (state) {
        case State::kInit: {
            m_intake.Deploy();
            state = State::kDriving;
            break;
        }
        case State::kDriving: {
            // Add a region constraint to slow down the drivetrain while
            // it's approaching the balls
            static frc::RectangularRegionConstraint regionConstraint{
                frc::Translation2d{endPose.X(),
                                   endPose.Y() - 0.5 * Drivetrain::kLength},
                frc::Translation2d{endPose.X() + 0.5 * Drivetrain::kLength,
                                   initialPose.Y() + 0.5 * Drivetrain::kLength},
                frc::MaxVelocityConstraint{1_mps}};

            if (lastState != state) {
                auto config = m_drivetrain.MakeTrajectoryConfig();
                config.AddConstraint(regionConstraint);
                m_drivetrain.SetWaypoints(initialPose, {}, endPose, config);

                lastState = state;
            }

            // Intake Balls x2
            if (regionConstraint.IsPoseInRegion(m_drivetrain.GetPose())) {
                m_intake.SetArmMotor(Intake::ArmMotorDirection::kIntake);
                m_intake.SetFunnel(0.4);
            }

            if (m_drivetrain.AtGoal()) {
                m_intake.SetArmMotor(Intake::ArmMotorDirection::kIdle);
                m_intake.SetFunnel(0);

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
