// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "logging/CSVUtil.hpp"

#if !defined(__FRC_ROBORIO__)
#if defined(__GNUC__) && __GNUC__ < 9 && !defined(__llvm__)
#include <experimental/filesystem>
namespace std {
namespace filesystem = experimental::filesystem;
}  // namespace std
#else
#include <filesystem>
#endif  // defined(__GNUC__) && __GNUC__ < 9 && !defined(__llvm__)
#endif  // !defined(__FRC_ROBORIO__)

#include <fmt/core.h>

namespace frc3512 {

constexpr bool starts_with(std::string_view str, std::string_view x) noexcept {
    return str.substr(0, x.size()) == x;
}

constexpr bool ends_with(std::string_view str, std::string_view x) noexcept {
    return str.size() >= x.size() &&
           str.compare(str.size() - x.size(), std::string_view::npos, x) == 0;
}

void DeleteCSVs() {
#if !defined(__FRC_ROBORIO__)
    namespace fs = std::filesystem;

    for (auto& p : fs::recursive_directory_iterator(".")) {
        if (ends_with(p.path().string(), ".csv")) {
            fs::remove(p.path());
        }
    }
#endif  // !defined(__FRC_ROBORIO__)
}

void AddPrefixToCSVs(std::string_view prefix) {
#if !defined(__FRC_ROBORIO__)
    namespace fs = std::filesystem;

    for (auto& p : fs::recursive_directory_iterator(".")) {
        auto src = p.path();
        auto filename = src.filename().string();

        if (!ends_with(filename, ".csv")) {
            continue;
        }

        auto filePrefix =
            std::string_view{filename}.substr(0, filename.find("-"));
        for (const auto& subsystem :
             {"Battery", "Drivetrain ", "Flywheel ", "Intake", "Turret "}) {
            if (starts_with(filePrefix, subsystem)) {
                auto dest = src;
                dest.replace_filename(fmt::format("{} {}", prefix, filename));
                fs::rename(src, dest);
            }
        }
    }
#endif  // !defined(__FRC_ROBORIO__)
}

}  // namespace frc3512
