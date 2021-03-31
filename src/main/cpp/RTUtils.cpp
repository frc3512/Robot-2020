// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include "RTUtils.hpp"

#if !defined(_MSC_VER)
#include <unistd.h>
#endif  // !defined(_MSC_VER)

#include <cstdlib>
#include <stdexcept>
#include <string>

#include <fmt/format.h>
#include <frc/RobotBase.h>

namespace frc3512 {

void SetProcessRTPriority(std::string_view process, int priority) {
    if constexpr (!frc::RobotBase::IsSimulation()) {
#if !defined(_MSC_VER)
        setuid(0);
#endif  // !defined(_MSC_VER)
        if (std::system(fmt::format("pgrep '{}' | xargs chrt -f -p {}", process,
                                    priority)
                            .c_str()) != 0) {
            throw std::runtime_error(fmt::format(
                "Setting {} to RT priority {} failed", process, priority));
        }
    }
}

}  // namespace frc3512
