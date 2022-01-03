// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>

namespace TargetModel {
// Target model-points in the global frame, pulled from
// https://files.slack.com/files-pri/T29CNG6MQ-FTV1L0XD1/img_4471.jpg
// New Target model-points in the global frame, pulled from
// https://firstfrc.blob.core.windows.net/frc2020/PlayingField/2020FieldDrawing-SeasonSpecific.pdf
// page 197 and 207

// An offset is added to the real target location so the robot aims for a point
// a few inches in front of the target.
extern const frc::Translation2d kOffset;
static constexpr frc::Translation3d kA{629.25_in, 108.464_in, 98.19_in};
static constexpr frc::Translation3d kB{629.25_in, 106.464_in, 98.19_in};
static constexpr frc::Translation3d kC{629.25_in, 98.644_in, 81.19_in};
static constexpr frc::Translation3d kD{629.25_in, 93.912_in, 83.19_in};
static constexpr frc::Translation3d kE{629.25_in, 80.75_in, 81.19_in};
static constexpr frc::Translation3d kF{629.25_in, 82.214_in, 83.19_in};
static constexpr frc::Translation3d kG{629.25_in, 69.214_in, 98.19_in};
static constexpr frc::Translation3d kH{629.25_in, 71.214_in, 98.19_in};
static constexpr frc::Translation3d kCenter = (kA + kG) / 2.0;
extern const frc::Pose2d kTargetPoseInGlobal;
}  // namespace TargetModel
