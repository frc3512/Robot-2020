/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <frc/system/LinearSystem.h>
#include <gtest/gtest.h>

#include <units.h>

#include "frc/system/plant/LinearSystemId.h"

TEST(LinearSystemIDTest, IdentifyVelocitySystem) {
  double kv = 1.0;
  double ka = 0.5;
  auto system = frc::LinearSystemId::IdentifyVelocitySystem(kv, ka);
}
