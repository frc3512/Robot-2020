// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/geometry/Translation3d.h>

namespace TargetModel {
// Target model-points in the global frame, pulled from
// https://files.slack.com/files-pri/T29CNG6MQ-FTV1L0XD1/img_4471.jpg
static constexpr frc::Translation3d kA{629.25_in, 108.464_in, 115.25_in};
static constexpr frc::Translation3d kB{629.25_in, 106.464_in, 115.25_in};
static constexpr frc::Translation3d kC{629.25_in, 98.644_in, 98.25_in};
static constexpr frc::Translation3d kD{629.25_in, 93.912_in, 100.25_in};
static constexpr frc::Translation3d kE{629.25_in, 80.75_in, 98.25_in};
static constexpr frc::Translation3d kF{629.25_in, 82.214_in, 100.25_in};
static constexpr frc::Translation3d kG{629.25_in, 69.214_in, 115.25_in};
static constexpr frc::Translation3d kH{629.25_in, 71.214_in, 115.25_in};

static constexpr frc::Translation3d kCenter = (kA + kG) / 2.0;
}  // namespace TargetModel
