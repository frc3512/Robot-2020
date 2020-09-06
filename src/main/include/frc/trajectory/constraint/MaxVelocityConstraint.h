/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <units/units.h>

#include "frc/trajectory/constraint/TrajectoryConstraint.h"

namespace frc {

/**
 * A constraint on the maximum velocity allowed when traversing a trajectory.
 */
class MaxVelocityConstraint : public TrajectoryConstraint {
 public:
  /**
   * Creates a new MaxVelocityConstraint.
   *
   * @param maxVelocity Maximum velocity of the trajectory.
   */
  explicit MaxVelocityConstraint(units::meters_per_second_t maxVelocity)
      : m_maxVelocity(units::math::abs(maxVelocity)) {}

  units::meters_per_second_t MaxVelocity(
      const Pose2d& pose, curvature_t curvature,
      units::meters_per_second_t velocity) override {
    return units::math::min(
        velocity, m_maxVelocity);  // NOLINT(build/include_what_you_use)
  }

  MinMax MinMaxAcceleration(const Pose2d& pose, curvature_t curvature,
                            units::meters_per_second_t speed) override {
    return {};
  }

 private:
  units::meters_per_second_t m_maxVelocity;
};

}  // namespace frc
