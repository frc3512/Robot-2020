// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include "TargetModel.hpp"

namespace TargetModel {
const frc::Translation2d kOffset{0_in, units::inch_t{5.0}};
const frc::Pose2d kTargetPoseInGlobal{TargetModel::kCenter.X(),
                                      TargetModel::kCenter.Y(),
                                      units::radian_t{wpi::math::pi}};
}  // namespace TargetModel
