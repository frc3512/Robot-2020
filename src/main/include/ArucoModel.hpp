// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>

namespace ArucoModel {
static const frc::Translation3d kTopLeft{4.6736_m, -0.365_m, 1.2224_m};
static const frc::Translation3d kTopRight{4.4704_m, -0.365_m, 1.2224_m};
static const frc::Translation3d kBotLeft{4.6736_m, -0.365_m, 1.0255_m};
static const frc::Translation3d kBotRight{4.4704_m, -0.365_m, 1.0255_m};
static const frc::Translation3d kCenter = (kTopLeft - kBotRight) / 2;
}  // namespace ArucoModel
