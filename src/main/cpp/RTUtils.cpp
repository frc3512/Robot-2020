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

namespace {
#if !defined(_MSC_VER)
class UidSetter {
public:
    explicit UidSetter(uid_t uid) {
        m_uid = getuid();
        if (setuid(uid) == -1) {
            throw std::system_error(errno, std::generic_category(),
                                    fmt::format("setuid({}) failed", uid));
        }
    }

    ~UidSetter() noexcept(false) {
        if (setuid(m_uid) == -1) {
            throw std::system_error(errno, std::generic_category(),
                                    fmt::format("setuid({}) failed", m_uid));
        }
    }

private:
    uid_t m_uid;
};
#else
class UidSetter {
public:
    explicit UidSetter(int uid) {}
};
#endif  // !defined(_MSC_VER)
}  // namespace

namespace frc3512 {

void SetProcessRTPriority(std::string_view process, int priority) {
    if constexpr (!frc::RobotBase::IsSimulation()) {
        UidSetter uidSetter{0};

        if (std::system(fmt::format("pgrep '{}' | xargs chrt -f -p {}", process,
                                    priority)
                            .c_str()) != 0) {
            throw std::runtime_error(fmt::format(
                "Setting {} to RT priority {} failed", process, priority));
        }
    }
}

}  // namespace frc3512
