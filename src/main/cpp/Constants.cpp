// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include "Constants.hpp"

namespace frc3512::Constants::Vision {
const char kCameraName[] = "gloworm";
const units::meter_t kCameraHeight = 39_in;
const units::degree_t kCameraPitch = 22.8_deg;
const units::degree_t kCameraDiagonalFOV = 74.8_deg;
const frc::Pose2d kDrivetrainToTurretFrame{2_in, 0_m,
                                           units::radian_t{wpi::math::pi}};
}  // namespace frc3512::Constants::Vision
