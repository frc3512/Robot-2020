// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

namespace {
enum class State { kInit, kTrenchRun, kIdle };
}  // namespace

static State state;
static frc2::Timer autonTimer;

static const frc::Pose2d initialPose{12.89_m, 7.513_m,
                                     units::radian_t{wpi::math::pi}};
static const frc::Pose2d endPose{9.40_m, 7.513_m,
                                 units::radian_t{wpi::math::pi}};

void Robot::AutoLeftSideIntakeInit() {
    wpi::outs() << "LeftSideIntake autonomous\n";

    m_drivetrain.Reset(initialPose);

    state = State::kInit;
    autonTimer.Reset();
    autonTimer.Start();
}

void Robot::AutoLeftSideIntakePeriodic() {
    switch (state) {
        case State::kInit: {
            // Add a constraint to slow down the drivetrain while it's
            // approaching the balls
            frc::Translation2d bottomLeftConstraint{11_m, initialPose.Y()};
            frc::Translation2d topRightConstraint{endPose.X(), endPose.Y()};
            frc::MaxVelocityConstraint velocityConstraint{1.5_mps};

            // Make a trajectory config to add the rectangle constraint
            auto config = m_drivetrain.MakeTrajectoryConfig();
            config.AddConstraint(frc::RectangularRegionConstraint{
                bottomLeftConstraint, topRightConstraint, velocityConstraint});

            // Inital Pose - X: 12.91 m Y: 7.5 m Heading: pi rad
            m_drivetrain.SetWaypoints(initialPose, {}, endPose, config);

            state = State::kTrenchRun;
            break;
        }
        case State::kTrenchRun: {
            // Intake Balls x2
            // TODO: Add if pose in region if statement
            m_intake.SetArmMotor(Intake::ArmMotorDirection::kIntake);
            m_intake.SetFunnel(0.4);
            state = State::kIdle;
            break;
        }
        case State::kIdle: {
            // Stop Intake
            // Final Pose -
            // X: 9.4 m  Y: 7.500 m  Heading: 1*pi rad
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
