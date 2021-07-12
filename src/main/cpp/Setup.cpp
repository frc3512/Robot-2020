// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include "Setup.hpp"

#if !defined(_MSC_VER)
#include <unistd.h>
#endif  // !defined(_MSC_VER)

#include <cstdlib>
#include <stdexcept>

#include <frc/RobotBase.h>
#include <fmt/format.h>

namespace frc3512 {

void StopCrond() {
    if constexpr (!frc::RobotBase::IsSimulation()) {
        // crond occasionally uses 50% CPU and there's no cronjobs to run
#if !defined(_MSC_VER)
        setuid(0);
#endif  // !defined(_MSC_VER)
        int status = std::system("/etc/init.d/crond stop");
        if (status != 0) {
            throw std::runtime_error(
                fmt::format("Failed to stop crond ({})", status));
        }
    }
}

}  // namespace frc3512
