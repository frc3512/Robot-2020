// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "Setup.hpp"

#if !defined(_MSC_VER)
#include <unistd.h>
#endif  // !defined(_MSC_VER)

#include <cerrno>
#include <cstdlib>
#include <stdexcept>
#include <system_error>

#include <frc/RobotBase.h>
#include <frc/UidSetter.h>
#include <fmt/format.h>
#include <fmt/os.h>

namespace frc3512 {

void SetRTRuntimeLimit() {
    if constexpr (!frc::RobotBase::IsSimulation()) {
        frc::UidSetter uidSetter{0};

        auto setting = fmt::output_file("/proc/sys/kernel/sched_rt_runtime_us");
        setting.print("950000\n");
    }
}

void StopCrond() {
    if constexpr (!frc::RobotBase::IsSimulation()) {
        frc::UidSetter uidSetter{0};

        int status = std::system("/etc/init.d/crond stop");
        if (status != 0) {
            throw std::runtime_error(
                fmt::format("Failed to stop crond ({})", status));
        }
    }
}

}  // namespace frc3512
