// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <vector>

#include <frc/geometry/Translation2d.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrapezoidProfile.h>

namespace frc3512 {

class TrajectoryUtil {
public:
    /**
     * Creates Trajectory to follow for turning in place.
     *
     * @param constraints Constraints for the trajectory
     * @param goal Goal to set the robot to turn
     * @param initial The angle of where the robot currently
     * @return Trajectory for turning in place
     */
    static frc::Trajectory TurnInPlaceTrajectory(
        frc::TrapezoidProfile<units::radian>::Constraints constraints,
        frc::TrapezoidProfile<units::radian>::State goal,
        frc::TrapezoidProfile<units::radian>::State initial,
        frc::Translation2d currentPosition);
};
}  // namespace frc3512
