/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <limits>
#include <tuple>

#include <units/units.h>

#include "frc/system/LinearSystem.h"
#include "frc/trajectory/constraint/TrajectoryConstraint.h"

namespace frc {
/**
 * A class that enforces constraints on differential drive voltage expenditure
 * based on the motor dynamics and the drive kinematics.  Ensures that the
 * acceleration of any wheel of the robot while following the trajectory is
 * never higher than what can be achieved with the given maximum voltage.
 */
class DrivetrainVelocitySystemConstraint : public TrajectoryConstraint {
 public:
  /**
   * Creates a new DrivetrainVelocitySystemConstraint.
   *
   * @param system     The drivetrain state-space model.
   * @param trackWidth The distance between the drivetrain's wheels.
   * @param maxVoltage The maximum voltage available to the motors while
   *                   following the path. Should be somewhat less than the
   *                   nominal battery voltage (12V) to account for "voltage
   *                   sag" due to current draw.
   */
  DrivetrainVelocitySystemConstraint(const LinearSystem<2, 2, 2>& system,
                                     units::meter_t trackWidth,
                                     units::volt_t maxVoltage)
      : m_system(&system), m_trackWidth(trackWidth), m_maxVoltage(maxVoltage) {}

  units::meters_per_second_t MaxVelocity(
      const Pose2d& pose, curvature_t curvature,
      units::meters_per_second_t velocity) override;

  MinMax MinMaxAcceleration(const Pose2d& pose, curvature_t curvature,
                            units::meters_per_second_t speed) override;

 private:
  const LinearSystem<2, 2, 2>* m_system;
  units::meter_t m_trackWidth;
  units::volt_t m_maxVoltage;

  /**
   * Converts velocity and curvature of drivetrain into left and right wheel
   * velocities.
   *
   * @param velocity   Linear velocity of drivetrain chassis.
   * @param curvature  Curvature of drivetrain arc.
   * @param trackWidth Track width of drivetrain.
   */
  static constexpr std::tuple<units::meters_per_second_t,
                              units::meters_per_second_t>
  ToWheelVelocities(units::meters_per_second_t velocity,
                    frc::curvature_t curvature, units::meter_t trackWidth) {
    // clang-format off
    // v = (v_r + v_l) / 2     (1)
    // w = (v_r - v_l) / (2r)  (2)
    // k = w / v               (3)
    //
    // v_l = v - wr
    // v_l = v - (vk)r
    // v_l = v(1 - kr)
    //
    // v_r = v + wr
    // v_r = v + (vk)r
    // v_r = v(1 + kr)
    // clang-format on
    auto vl = velocity * (1 - (curvature / 1_rad * trackWidth / 2.0));
    auto vr = velocity * (1 + (curvature / 1_rad * trackWidth / 2.0));
    return {vl, vr};
  }
};
}  // namespace frc
