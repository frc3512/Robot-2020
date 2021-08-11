// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/UidSetter.h"

#if defined(__FRC_ROBORIO__)
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <stdexcept>
#include <system_error>

#include <fmt/format.h>
#endif  // defined(__FRC_ROBORIO__)

using namespace frc;

UidSetter::UidSetter(int uid) {
#if defined(__FRC_ROBORIO__)
  m_uid = geteuid();
  if (uid == 0 && setuid(uid) == -1) {
    throw std::system_error(errno, std::generic_category(),
                            fmt::format("setuid({}) failed", uid));
  } else if (uid != 0 && seteuid(uid) == -1) {
    throw std::system_error(errno, std::generic_category(),
                            fmt::format("seteuid({}) failed", uid));
  }
#endif  // defined(__FRC_ROBORIO__)
}

UidSetter::~UidSetter() noexcept(false) {
#if defined(__FRC_ROBORIO__)
  if (seteuid(m_uid) == -1) {
    throw std::system_error(errno, std::generic_category(),
                            fmt::format("seteuid({}) failed", m_uid));
  }
#endif  // defined(__FRC_ROBORIO__)
}
