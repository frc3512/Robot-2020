// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "TargetModel.hpp"

#include <wpi/numbers>

namespace TargetModel {
const frc::Translation2d kOffset{0_in, units::inch_t{5.411}};
const frc::Pose2d kTargetPoseInGlobal{TargetModel::kCenter.X(),
                                      TargetModel::kCenter.Y(),
                                      units::radian_t{wpi::numbers::pi}};
}  // namespace TargetModel
