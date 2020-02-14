// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <wpi/math>

#include "Robot.hpp"

namespace frc3512 {

enum class State { kInit, kTrenchRun, kIdle };
static State state;

void Robot::AutoRightSideIntakeInit() { state = State::kInit; }

void Robot::AutoRightSideIntakePeriodic() {
    switch (state) {
        case State::kInit: {
            // Initial Pose - X: 12.65 m  Y: 0.7500 m  Heading: 1*pi rad
            m_drivetrain.Reset(
                frc::Pose2d(12.65_m, 0.7500_m, units::radian_t{wpi::math::pi}));
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
}

}  // namespace frc3512
