// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "CSVTestUtil.hpp"

#include <cstdio>
#include <experimental/filesystem>
#include <string>
#include <vector>

namespace frc3512 {
void AddPrefixToCSVs(std::string_view prefix) {
    std::vector<std::string> names{"./Drivetrain ", "./Flywheel ", "./Turret "};
    for (auto& p : std::experimental::filesystem::directory_iterator(".")) {
        std::string directoryEntry = p.path().u8string();
        std::string oldFilename =
            directoryEntry.substr(directoryEntry.find("/") + 1);
        for (const auto& name : names) {
            if (directoryEntry.find(name) == 0) {
                std::string newFilename =
                    std::string{prefix.data()} + " " + oldFilename;
                std::rename(oldFilename.c_str(), newFilename.c_str());
            }
        }
    }
}
}  // namespace frc3512
