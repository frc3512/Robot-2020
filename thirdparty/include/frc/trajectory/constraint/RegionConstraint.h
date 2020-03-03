/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <limits>

#include <units/units.h>

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/geometry/Translation2d.h"
#include "frc/trajectory/constraint/TrajectoryConstraint.h"

namespace frc {

/**
 * A class that enforces constraints when the trajectory pose in a specific
 * ellipse region.
 */
class RegionConstraint : public TrajectoryConstraint {
 public:
  /**
   * Creates a new RegionConstraint.
   *
   * @param origin      The center of the region
   * @param xWidth      The width of the region in the x direction
   * @param yWidth      The width of the region in the y direction
   * @param constraint  The constraint to follow if the trajectory's current
   *                    pose is within the region
   * @param rotation    The rotation to apply to our radii around the origin
   */
  RegionConstraint(const Translation2d& origin, units::meter_t xWidth,
                   units::meter_t yWidth, TrajectoryConstraint& constraint,
                   const Rotation2d& rotation = 0_rad)
      : m_origin(origin),
        m_radii(xWidth / 2.0, yWidth / 2.0),
        m_constraint(constraint) {
    m_radii = m_radii.RotateBy(rotation);
  }

  units::meters_per_second_t MaxVelocity(
      const Pose2d& pose, curvature_t curvature,
      units::meters_per_second_t velocity) override {
    if (IsPoseInRegion(pose)) {
      return m_constraint.MaxVelocity(pose, curvature, velocity);
    } else {
      return units::meters_per_second_t{
          std::numeric_limits<double>::infinity()};
    }
  }

  MinMax MinMaxAcceleration(const Pose2d& pose, curvature_t curvature,
                            units::meters_per_second_t speed) override {
    if (IsPoseInRegion(pose)) {
      return m_constraint.MinMaxAcceleration(pose, curvature, speed);
    } else {
      return {};
    }
  }

  /**
   * Returns whether the given pose is within the elliptical region.
   *
   * @param pose The pose with which to test intersection.
   */
  bool IsPoseInRegion(const Pose2d& pose) const {
    // The region (disk) bounded by the ellipse is given by the equation:
    // ((x-h)^2)/Rx^2) + ((y-k)^2)/Ry^2) <= 1
    // If the inequality is satisfied, then it is inside the ellipse; otherwise
    // it is outside the ellipse.
    // Both sides have been multiplied by Rx^2 * Ry^2 for efficiency reasons.
    return units::math::pow<2>(pose.Translation().X() - m_origin.X()) *
                   units::math::pow<2>(m_radii.Y()) +
               units::math::pow<2>(pose.Translation().Y() - m_origin.Y()) *
                   units::math::pow<2>(m_radii.X()) <=
           units::math::pow<2>(m_radii.X()) * units::math::pow<2>(m_radii.Y());
  }

 private:
  Translation2d m_origin;
  Translation2d m_radii;
  TrajectoryConstraint& m_constraint;
};

}  // namespace frc
