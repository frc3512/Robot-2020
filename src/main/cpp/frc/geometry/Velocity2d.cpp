// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/geometry/Velocity2d.h"

#include <wpi/json.h>

#include "units/math.h"

using namespace frc;

Velocity2d::Velocity2d(units::meters_per_second_t x,
                       units::meters_per_second_t y)
    : m_x(x), m_y(y) {}

Velocity2d::Velocity2d(units::meters_per_second_t distance,
                       const Rotation2d& angle)
    : m_x(distance * angle.Cos()), m_y(distance * angle.Sin()) {}

units::meters_per_second_t Velocity2d::Distance(const Velocity2d& other) const {
  return units::math::hypot(other.m_x - m_x, other.m_y - m_y);
}

units::meters_per_second_t Velocity2d::Norm() const {
  return units::math::hypot(m_x, m_y);
}

Velocity2d Velocity2d::RotateBy(const Rotation2d& other) const {
  return {m_x * other.Cos() - m_y * other.Sin(),
          m_x * other.Sin() + m_y * other.Cos()};
}

Velocity2d Velocity2d::operator+(const Velocity2d& other) const {
  return {X() + other.X(), Y() + other.Y()};
}

Velocity2d& Velocity2d::operator+=(const Velocity2d& other) {
  m_x += other.m_x;
  m_y += other.m_y;
  return *this;
}

Velocity2d Velocity2d::operator-(const Velocity2d& other) const {
  return *this + -other;
}

Velocity2d& Velocity2d::operator-=(const Velocity2d& other) {
  *this += -other;
  return *this;
}

Velocity2d Velocity2d::operator-() const { return {-m_x, -m_y}; }

Velocity2d Velocity2d::operator*(double scalar) const {
  return {scalar * m_x, scalar * m_y};
}

Velocity2d& Velocity2d::operator*=(double scalar) {
  m_x *= scalar;
  m_y *= scalar;
  return *this;
}

Velocity2d Velocity2d::operator/(double scalar) const {
  return *this * (1.0 / scalar);
}

bool Velocity2d::operator==(const Velocity2d& other) const {
  return units::math::abs(m_x - other.m_x) < 1E-9_mps &&
         units::math::abs(m_y - other.m_y) < 1E-9_mps;
}

bool Velocity2d::operator!=(const Velocity2d& other) const {
  return !operator==(other);
}

Velocity2d& Velocity2d::operator/=(double scalar) {
  *this *= (1.0 / scalar);
  return *this;
}

void frc::to_json(wpi::json& json, const Velocity2d& velocity) {
  json = wpi::json{{"x", velocity.X().value()}, {"y", velocity.Y().value()}};
}

void frc::from_json(const wpi::json& json, Velocity2d& velocity) {
  velocity = Velocity2d{units::meters_per_second_t{json.at("x").get<double>()},
                        units::meters_per_second_t{json.at("y").get<double>()}};
}
