// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

enum class State { kInit, kShoot, kTrenchRun, kTrenchShoot, kIdle };
static State state;

void Robot::AutoRightSideShootSixInit() { state = State::kInit; }

void Robot::AutoRightSideShootSixPeriodic() {
#if 0
    switch (state) {
        case State::kInit: {
            // Initial Pose - X: 12.65 m  Y: 0.7500 m  Heading: 0 rad
            m_drivetrain.Reset(frc::Pose2d(12.65_m, 0.7500_m, 0_rad));
            m_drivetrain.SetWaypoints(frc::Pose2d(
                12.65_m - Constants::Drivetrain::kLength, 0.7500_m, 0_rad),
                {frc::Translation2d(-1.0, 0.0), frc::Translation2d(-3.875, 0.0)},
                frc::Pose2d(7.775_m + Constants::Drivetrain::kLength, 0.7500, units::radian_t{wpi::math::pi}));
            // Set constraints
            state = State::kShoot;
            break;
        }
        case State::kShoot: {
            // Shoot x3
            // Change if-statement condition to suit constraint
            if (m_drivetrain.AtGoal()) {
                m_flywheel.Shoot();
                timer.Reset();
                timer.Start();
            state = State::kTrenchRun;
            }
            break;
        }
        case State::kTrenchRun: {
            // Intake Balls x3
            // Final Pose -
            // X: 7.775 m + RobotLength  Y: 0.7500 m  Heading: 1*pi rad
            // Shoot x3
            if (timer.HasElapsed(3.0)) {
                m_intake.SetArmMotor(ArmMotorDirection::kIntake);
                m_intake.SetConveyor(0.85);
            state = State::kTrenchShoot;
            }
            break;
        }
        case State::kTrenchShoot: {
            if (m_drivetrain.AtGoal()) {
                m_intake.SetArmMotor(ArmMotorDirection::kIdle);
                m_intake.SetConveyor(0.85);
                m_flywheel.Shoot();
            state = State::kIdle;
            }
            break;
        }
        case State::kIdle: {
            if (timer.HasElapsed(10.0)) {
                m_intake.SetConveyor(0.0);
                timer.Stop();
            }
            break;
        }
    }
#endif
}

}  // namespace frc3512
