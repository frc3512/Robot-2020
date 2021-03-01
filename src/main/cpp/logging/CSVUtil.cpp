// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#include "logging/CSVUtil.hpp"

#include <algorithm>

#if !defined(__FRC_ROBORIO__)
#if defined(__GNUC__) && __GNUC__ < 8 && !defined(__llvm__)
#include <experimental/filesystem>
namespace std {
namespace filesystem = experimental::filesystem;
}  // namespace std
#else
#include <filesystem>
#endif  // defined(__GNUC__) && __GNUC__ < 8 && !defined(__llvm__)
#endif  // !defined(__FRC_ROBORIO__)

#include <frc2/Timer.h>

namespace frc3512 {

constexpr bool ends_with(std::string_view str, std::string_view x) noexcept {
    return str.size() >= x.size() &&
           str.compare(str.size() - x.size(), std::string_view::npos, x) == 0;
}

void DeleteCSVs() {
#if !defined(__FRC_ROBORIO__)
    namespace fs = std::filesystem;

    // Delete CSV files
    for (auto& p : fs::recursive_directory_iterator(".")) {
        if (!p.is_directory() && ends_with(p.path().string(), ".csv")) {
            fs::remove(p.path());
        }
    }

    // Delete empty directories
    for (auto& p : fs::directory_iterator(".")) {
        if (p.is_directory()) {
            auto childPath = fs::recursive_directory_iterator(p.path());
            auto numFiles =
                std::count_if(fs::begin(childPath), fs::end(childPath),
                              [](const auto& p) { return !p.is_directory(); });
            if (numFiles == 0) {
                fs::remove_all(p.path());
            }
        }
    }
#endif  // !defined(__FRC_ROBORIO__)
}

}  // namespace frc3512
