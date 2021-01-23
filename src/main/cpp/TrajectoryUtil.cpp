// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include "TrajectoryUtil.hpp"

#include <fmt/core.h>
#include <frc/logging/CSVLogFile.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/voltage.h>

#include "Constants.hpp"

using namespace frc3512;

frc::Trajectory TrajectoryUtil::TurnInPlaceTrajectory(
    frc::TrapezoidProfile<units::radian>::Constraints constraints,
    frc::TrapezoidProfile<units::radian>::State goal,
    frc::TrapezoidProfile<units::radian>::State initial,
    frc::Translation2d currentPosition) {
    frc::TrapezoidProfile<units::radian> profile{constraints, goal, initial};
    std::vector<frc::Trajectory::State> trajectoryStates;

    units::radians_per_second_t pastVelocity;
    units::radians_per_second_squared_t acceleration;
    for (auto t = 0_s; t < profile.TotalTime(); t += Constants::kDt) {
        auto state = profile.Calculate(t);
        if (state.velocity > pastVelocity) {
            acceleration = constraints.maxAcceleration;
        } else if (state.velocity < pastVelocity) {
            acceleration = -constraints.maxAcceleration;
        } else {
            acceleration = 0_rad_per_s_sq;
        }

        trajectoryStates.emplace_back(frc::Trajectory::State{
            t, 0_mps, 0_mps_sq,
            frc::Pose2d{currentPosition.X(), currentPosition.Y(),
                        state.position},
            units::curvature_t{std::numeric_limits<double>::infinity()}});
        pastVelocity = state.velocity;
    }

    frc::Trajectory traj = frc::Trajectory(trajectoryStates);
    frc::CSVLogFile trajectoryLogger{"trapezoid", "Time", "Rotation (rad)"};
    for (auto tra : traj.States()) {
        fmt::print(
            stderr,
            "Time: {0} Current X: {1} Current Y: {2} Current Theta: {3}\n",
            tra.t(), tra.pose.X().to<double>(), tra.pose.Y().to<double>(),
            tra.pose.Rotation().Radians().to<double>());
    }
    return traj;
}
