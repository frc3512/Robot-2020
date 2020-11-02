// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <limits>
#include <memory>

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/trajectory/constraint/TrajectoryConstraint.h>

namespace frc {
/**
 * Enforces a particular constraint only within a rectangular region.
 */
template <typename Constraint, typename = std::enable_if_t<std::is_base_of_v<
                                   TrajectoryConstraint, Constraint>>>
class RectangularRegionConstraint : public TrajectoryConstraint {
public:
    /**
     * Constructs a new RectangularRegionConstraint.
     *
     * @param bottomLeftPoint The bottom left point of the rectangular region in
     * which to enforce the constraint.
     * @param topRightPoint The top right point of the rectangular region in
     * which to enforce the constraint.
     * @param constraint The constraint to enforce when the robot is within the
     * region.
     */
    RectangularRegionConstraint(const Translation2d& bottomLeftPoint,
                                const Translation2d& topRightPoint,
                                Constraint constraint)
        : m_bottomLeftPoint(bottomLeftPoint),
          m_topRightPoint(topRightPoint),
          m_constraint(std::make_unique<Constraint>(constraint)) {}

    RectangularRegionConstraint(const RectangularRegionConstraint& rhs)
        : TrajectoryConstraint(rhs) {
        m_bottomLeftPoint = rhs.m_bottomLeftPoint;
        m_topRightPoint = rhs.m_topRightPoint;
        m_constraint = std::make_unique<Constraint>(*(rhs.m_constraint));
    }

    RectangularRegionConstraint& operator=(
        const RectangularRegionConstraint& rhs) {
        m_bottomLeftPoint = rhs.m_bottomLeftPoint;
        m_topRightPoint = rhs.m_topRightPoint;
        m_constraint.reset(new Constraint(*(rhs.m_constraint)));
        return *this;
    }

    RectangularRegionConstraint(RectangularRegionConstraint&&) = default;
    RectangularRegionConstraint& operator=(RectangularRegionConstraint&&) =
        default;

    units::meters_per_second_t MaxVelocity(
        const Pose2d& pose, units::curvature_t curvature,
        units::meters_per_second_t velocity) const override {
        if (IsPoseInRegion(pose)) {
            return m_constraint->MaxVelocity(pose, curvature, velocity);
        } else {
            return units::meters_per_second_t(
                std::numeric_limits<double>::infinity());
        }
    }

    MinMax MinMaxAcceleration(const Pose2d& pose, units::curvature_t curvature,
                              units::meters_per_second_t speed) const override {
        if (IsPoseInRegion(pose)) {
            return m_constraint->MinMaxAcceleration(pose, curvature, speed);
        } else {
            return {};
        }
    }

    /**
     * Returns whether the specified robot pose is within the region that the
     * constraint is enforced in.
     *
     * @param pose The robot pose.
     * @return Whether the robot pose is within the constraint region.
     */
    bool IsPoseInRegion(const Pose2d& pose) const {
        return pose.X() >= m_bottomLeftPoint.X() &&
               pose.X() <= m_topRightPoint.X() &&
               pose.Y() >= m_bottomLeftPoint.Y() &&
               pose.Y() <= m_topRightPoint.Y();
    }

private:
    Translation2d m_bottomLeftPoint;
    Translation2d m_topRightPoint;
    std::unique_ptr<Constraint> m_constraint;
};
}  // namespace frc
