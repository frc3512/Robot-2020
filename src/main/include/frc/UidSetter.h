// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

namespace frc {

/**
 * RAII wrapper for setting and restoring the effective user ID.
 */
class UidSetter {
 public:
  explicit UidSetter(int uid);

  ~UidSetter() noexcept(false);

 private:
#if defined(__FRC_ROBORIO__)
  int m_uid;
#endif  // defined(__FRC_ROBORIO__)
};

}  // namespace frc
