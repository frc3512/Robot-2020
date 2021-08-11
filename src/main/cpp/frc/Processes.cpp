// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/Processes.h"

#if !defined(_MSC_VER)
#include <unistd.h>
#endif  // !defined(_MSC_VER)

#include <cerrno>
#include <cstdlib>
#include <system_error>

#include <fmt/format.h>

#include "frc/RobotBase.h"
#include "frc/UidSetter.h"

namespace frc {

bool SetProcessPriority(std::string_view process, bool realTime, int priority) {
  if constexpr (!frc::RobotBase::IsSimulation()) {
    UidSetter uidSetter{0};

    if (!realTime) {
      // Set scheduling policy to SCHED_OTHER
      if (std::system(
              fmt::format("pgrep '{}' | xargs chrt -o", process).c_str()) !=
          0) {
        return false;
      }
    } else {
      // Set scheduling policy to SCHED_FIFO with the given priority
      if (std::system(
              fmt::format("pgrep '{}' | xargs chrt -f -p {}", process, priority)
                  .c_str()) != 0) {
        return false;
      }
    }
  }

  return true;
}

}  // namespace frc
