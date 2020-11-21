// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/geometry/Translation3d.h>

namespace TargetModel {
// Target model-points in the global frame, pulled from
// https://files.slack.com/files-pri/T29CNG6MQ-FTV1L0XD1/img_4471.jpg

// An offset is added to the real target location so the robot aims for a point
// a few inches in front of the target.
static constexpr frc::Translation3d kOffset{units::inch_t{-5.0}, 0_in, 0_in};
static constexpr auto kA =
    frc::Translation3d{629.25_in, 108.464_in, 115.25_in} + kOffset;
static constexpr auto kB =
    frc::Translation3d{629.25_in, 106.464_in, 115.25_in} + kOffset;
static constexpr auto kC =
    frc::Translation3d{629.25_in, 98.644_in, 98.25_in} + kOffset;
static constexpr auto kD =
    frc::Translation3d{629.25_in, 93.912_in, 100.25_in} + kOffset;
static constexpr auto kE =
    frc::Translation3d{629.25_in, 80.75_in, 98.25_in} + kOffset;
static constexpr auto kF =
    frc::Translation3d{629.25_in, 82.214_in, 100.25_in} + kOffset;
static constexpr auto kG =
    frc::Translation3d{629.25_in, 69.214_in, 115.25_in} + kOffset;
static constexpr auto kH =
    frc::Translation3d{629.25_in, 71.214_in, 115.25_in} + kOffset;
static constexpr auto kCenter = (kA + kG) / 2.0;
}  // namespace TargetModel
