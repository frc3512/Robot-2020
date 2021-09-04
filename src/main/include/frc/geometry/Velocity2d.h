// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/geometry/Rotation2d.h"
#include "units/velocity.h"

namespace wpi {
class json;
}  // namespace wpi

namespace frc {

/**
 * Represents a velocity in 2d space.
 * This object can be used to represent a point or a vector.
 *
 * This assumes that you are using conventional mathematical axes.
 * When the robot is placed on the origin, facing toward the X direction,
 * moving forward increases the X, whereas moving to the left increases the Y.
 */
class Velocity2d {
 public:
  /**
   * Constructs a Velocity2d with X and Y components equal to zero.
   */
  constexpr Velocity2d() = default;

  /**
   * Constructs a Velocity2d with the X and Y components equal to the
   * provided values.
   *
   * @param x The x component of the velocity.
   * @param y The y component of the velocity.
   */
  Velocity2d(units::meters_per_second_t x, units::meters_per_second_t y);

  /**
   * Constructs a Velocity2d with the provided velocity and angle. This is
   * essentially converting from polar coordinates to Cartesian coordinates.
   *
   * @param velocity The velocity from the origin to the end of the velocity.
   * @param angle The angle between the x-axis and the velocity vector.
   */
  Velocity2d(units::meters_per_second_t velocity, const Rotation2d& angle);

  /**
   * Calculates the velocity between two velocities in 2d space.
   *
   * This function uses the pythagorean theorem to calculate the velocity.
   * velocity = std::sqrt((x2 - x1)^2 + (y2 - y1)^2)
   *
   * @param other The velocity to compute the velocity to.
   *
   * @return The velocity between the two velocities.
   */
  units::meters_per_second_t Distance(const Velocity2d& other) const;

  /**
   * Returns the X component of the velocity.
   *
   * @return The x component of the velocity.
   */
  units::meters_per_second_t X() const { return m_x; }

  /**
   * Returns the Y component of the velocity.
   *
   * @return The y component of the velocity.
   */
  units::meters_per_second_t Y() const { return m_y; }

  /**
   * Returns the norm, or velocity from the origin to the velocity.
   *
   * @return The norm of the velocity.
   */
  units::meters_per_second_t Norm() const;

  /**
   * Applies a rotation to the velocity in 2d space.
   *
   * This multiplies the velocity vector by a counterclockwise rotation
   * matrix of the given angle.
   *
   * [x_new]   [other.cos, -other.sin][x]
   * [y_new] = [other.sin,  other.cos][y]
   *
   * For example, rotating a Velocity2d of {2, 0} by 90 degrees will return a
   * Velocity2d of {0, 2}.
   *
   * @param other The rotation to rotate the velocity by.
   *
   * @return The new rotated velocity.
   */
  Velocity2d RotateBy(const Rotation2d& other) const;

  /**
   * Adds two velocities in 2d space and returns the sum. This is similar to
   * vector addition.
   *
   * For example, Velocity2d{1.0, 2.5} + Velocity2d{2.0, 5.5} =
   * Velocity2d{3.0, 8.0}
   *
   * @param other The velocity to add.
   *
   * @return The sum of the velocities.
   */
  Velocity2d operator+(const Velocity2d& other) const;

  /**
   * Adds the new velocity to the current velocity.
   *
   * This is similar to the + operator, except that the current object is
   * mutated.
   *
   * @param other The velocity to add.
   *
   * @return The reference to the new mutated object.
   */
  Velocity2d& operator+=(const Velocity2d& other);

  /**
   * Subtracts the other velocity from the other velocity and returns the
   * difference.
   *
   * For example, Velocity2d{5.0, 4.0} - Velocity2d{1.0, 2.0} =
   * Velocity2d{4.0, 2.0}
   *
   * @param other The velocity to subtract.
   *
   * @return The difference between the two velocities.
   */
  Velocity2d operator-(const Velocity2d& other) const;

  /**
   * Subtracts the new velocity from the current velocity.
   *
   * This is similar to the - operator, except that the current object is
   * mutated.
   *
   * @param other The velocity to subtract.
   *
   * @return The reference to the new mutated object.
   */
  Velocity2d& operator-=(const Velocity2d& other);

  /**
   * Returns the inverse of the current velocity. This is equivalent to
   * rotating by 180 degrees, flipping the point over both axes, or simply
   * negating both components of the velocity.
   *
   * @return The inverse of the current velocity.
   */
  Velocity2d operator-() const;

  /**
   * Multiplies the velocity by a scalar and returns the new velocity.
   *
   * For example, Velocity2d{2.0, 2.5} * 2 = Velocity2d{4.0, 5.0}
   *
   * @param scalar The scalar to multiply by.
   *
   * @return The scaled velocity.
   */
  Velocity2d operator*(double scalar) const;

  /**
   * Multiplies the current velocity by a scalar.
   *
   * This is similar to the * operator, except that current object is mutated.
   *
   * @param scalar The scalar to multiply by.
   *
   * @return The reference to the new mutated object.
   */
  Velocity2d& operator*=(double scalar);

  /**
   * Divides the velocity by a scalar and returns the new velocity.
   *
   * For example, Velocity2d{2.0, 2.5} / 2 = Velocity2d{1.0, 1.25}
   *
   * @param scalar The scalar to divide by.
   *
   * @return The scaled velocity.
   */
  Velocity2d operator/(double scalar) const;

  /**
   * Checks equality between this Velocity2d and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are equal.
   */
  bool operator==(const Velocity2d& other) const;

  /**
   * Checks inequality between this Velocity2d and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are not equal.
   */
  bool operator!=(const Velocity2d& other) const;

  /**
   * Divides the current velocity by a scalar.
   *
   * This is similar to the / operator, except that current object is mutated.
   *
   * @param scalar The scalar to divide by.
   *
   * @return The reference to the new mutated object.
   */
  Velocity2d& operator/=(double scalar);

 private:
  units::meters_per_second_t m_x = 0_mps;
  units::meters_per_second_t m_y = 0_mps;
};

void to_json(wpi::json& json, const Velocity2d& state);

void from_json(const wpi::json& json, Velocity2d& state);

}  // namespace frc
