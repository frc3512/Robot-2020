// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "frc/geometry/Rotation2d.h"
#include "units/length.h"
#include "units/math.h"

namespace wpi {
class json;
}  // namespace wpi

namespace frc {

/**
 * Represents a translation in 3d space.
 * This object can be used to represent a point or a vector.
 *
 * This assumes that you are using conventional mathematical axes.
 * When the robot is placed on the origin, facing toward the X direction,
 * moving forward increases the X, whereas moving to the left increases the Y.
 */
class Translation3d {
 public:
  /**
   * Constructs a Translation3d with X, Y, and Z components equal to zero.
   */
  constexpr Translation3d() = default;

  /**
   * Constructs a Translation3d with the X, Y, and Z components equal to the
   * provided values.
   *
   * @param x The x component of the translation.
   * @param y The y component of the translation.
   * @param z The z component of the translation.
   */
  constexpr Translation3d(units::meter_t x, units::meter_t y, units::meter_t z)
      : m_x(x), m_y(y), m_z(z) {}

  /**
   * Calculates the distance between two translations in 3d space.
   *
   * This function uses the pythagorean theorem to calculate the distance.
   * distance = std::sqrt((x2 - x1)^2 + (y2 - y1)^2 + (z2 - z1)^2)
   *
   * @param other The translation to compute the distance to.
   *
   * @return The distance between the two translations.
   */
  units::meter_t Distance(const Translation3d& other) const;

  /**
   * Returns the X component of the translation.
   *
   * @return The x component of the translation.
   */
  constexpr units::meter_t X() const { return m_x; }

  /**
   * Returns the Y component of the translation.
   *
   * @return The y component of the translation.
   */
  constexpr units::meter_t Y() const { return m_y; }

  /**
   * Returns the Z component of the translation.
   *
   * @return The z component of the translation.
   */
  constexpr units::meter_t Z() const { return m_z; }

  /**
   * Returns the norm, or distance from the origin to the translation.
   *
   * @return The norm of the translation.
   */
  units::meter_t Norm() const;

  /**
   * Applies a rotation to the translation in 3d space around the z axis.
   *
   * This multiplies the translation vector by a counterclockwise rotation
   * matrix of the given angle.
   *
   * [x_new]   [other.cos, -other.sin][x]
   * [y_new] = [other.sin,  other.cos][y]
   *
   * For example, rotating a Translation3d of {2, 0} by 90 degrees will return a
   * Translation3d of {0, 2}.
   *
   * @param other The rotation to rotate the translation by.
   *
   * @return The new rotated translation.
   */
  Translation3d RotateBy(const Rotation2d& other) const;

  /**
   * Adds two translations in 3d space and returns the sum. This is similar to
   * vector addition.
   *
   * For example, Translation3d{1.0, 2.5} + Translation3d{2.0, 5.5} =
   * Translation3d{3.0, 8.0}
   *
   * @param other The translation to add.
   *
   * @return The sum of the translations.
   */
  constexpr Translation3d operator+(const Translation3d& other) const {
    return {X() + other.X(), Y() + other.Y(), Z() + other.Z()};
  }

  /**
   * Adds the new translation to the current translation.
   *
   * This is similar to the + operator, except that the current object is
   * mutated.
   *
   * @param other The translation to add.
   *
   * @return The reference to the new mutated object.
   */
  Translation3d& operator+=(const Translation3d& other);

  /**
   * Subtracts the other translation from the other translation and returns the
   * difference.
   *
   * For example, Translation3d{5.0, 4.0} - Translation3d{1.0, 2.0} =
   * Translation3d{4.0, 2.0}
   *
   * @param other The translation to subtract.
   *
   * @return The difference between the two translations.
   */
  constexpr Translation3d operator-(const Translation3d& other) const {
    return *this + -other;
  }

  /**
   * Subtracts the new translation from the current translation.
   *
   * This is similar to the - operator, except that the current object is
   * mutated.
   *
   * @param other The translation to subtract.
   *
   * @return The reference to the new mutated object.
   */
  Translation3d& operator-=(const Translation3d& other);

  /**
   * Returns the inverse of the current translation. This is equivalent to
   * rotating by 180 degrees, flipping the point over both axes, or simply
   * negating both components of the translation.
   *
   * @return The inverse of the current translation.
   */
  constexpr Translation3d operator-() const {
    return {-1.0 * m_x, -1.0 * m_y, -1.0 * m_z};
  }

  /**
   * Multiplies the translation by a scalar and returns the new translation.
   *
   * For example, Translation3d{2.0, 2.5} * 2 = Translation3d{4.0, 5.0}
   *
   * @param scalar The scalar to multiply by.
   *
   * @return The scaled translation.
   */
  constexpr Translation3d operator*(double scalar) const {
    return {scalar * m_x, scalar * m_y, scalar * m_z};
  }

  /**
   * Multiplies the current translation by a scalar.
   *
   * This is similar to the * operator, except that current object is mutated.
   *
   * @param scalar The scalar to multiply by.
   *
   * @return The reference to the new mutated object.
   */
  Translation3d& operator*=(double scalar);

  /**
   * Divides the translation by a scalar and returns the new translation.
   *
   * For example, Translation3d{2.0, 2.5} / 2 = Translation3d{1.0, 1.25}
   *
   * @param scalar The scalar to divide by.
   *
   * @return The scaled translation.
   */
  constexpr Translation3d operator/(double scalar) const {
    return *this * (1.0 / scalar);
  }

  /**
   * Checks equality between this Translation3d and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are equal.
   */
  bool operator==(const Translation3d& other) const;

  /**
   * Checks inequality between this Translation3d and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are not equal.
   */
  bool operator!=(const Translation3d& other) const;

  /*
   * Divides the current translation by a scalar.
   *
   * This is similar to the / operator, except that current object is mutated.
   *
   * @param scalar The scalar to divide by.
   *
   * @return The reference to the new mutated object.
   */
  Translation3d& operator/=(double scalar);

 private:
  units::meter_t m_x = 0_m;
  units::meter_t m_y = 0_m;
  units::meter_t m_z = 0_m;
};

void to_json(wpi::json& json, const Translation3d& translation);

void from_json(const wpi::json& json, Translation3d& translation);

}  // namespace frc
