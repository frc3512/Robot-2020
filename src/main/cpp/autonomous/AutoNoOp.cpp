// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoNoOp() {
    m_turret.SetControlMode(TurretController::ControlMode::kManual);

    if constexpr (IsSimulation()) {
        EXPECT_EQ(0_V, m_turret.GetMotorOutput());
    }
}

}  // namespace frc3512
