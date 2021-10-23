// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/geometry/Translation3d.h"

#include <wpi/json.h>

using namespace frc;

units::meter_t Translation3d::Distance(const Translation3d& other) const {
  return (other - *this).Norm();
}

units::meter_t Translation3d::Norm() const {
  return units::math::sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
}

Translation3d Translation3d::RotateBy(const Rotation2d& other) const {
  return {m_x * other.Cos() - m_y * other.Sin(),
          m_x * other.Sin() + m_y * other.Cos(), m_z};
}

Translation3d& Translation3d::operator+=(const Translation3d& other) {
  m_x += other.m_x;
  m_y += other.m_y;
  m_z += other.m_z;
  return *this;
}

Translation3d& Translation3d::operator-=(const Translation3d& other) {
  *this += -other;
  return *this;
}

Translation3d& Translation3d::operator*=(double scalar) {
  m_x *= scalar;
  m_y *= scalar;
  m_z *= scalar;
  return *this;
}

bool Translation3d::operator==(const Translation3d& other) const {
  return units::math::abs(m_x - other.m_x) < 1E-9_m &&
         units::math::abs(m_y - other.m_y) < 1E-9_m &&
         units::math::abs(m_z - other.m_z) < 1E-9_m;
}

bool Translation3d::operator!=(const Translation3d& other) const {
  return !operator==(other);
}

Translation3d& Translation3d::operator/=(double scalar) {
  *this *= (1.0 / scalar);
  return *this;
}

void frc::to_json(wpi::json& json, const Translation3d& translation) {
  json = wpi::json{{"x", translation.X().value()},
                   {"y", translation.Y().value()},
                   {"z", translation.Z().value()}};
}

void frc::from_json(const wpi::json& json, Translation3d& translation) {
  translation = Translation3d{units::meter_t{json.at("x").get<double>()},
                              units::meter_t{json.at("y").get<double>()},
                              units::meter_t{json.at("z").get<double>()}};
}
