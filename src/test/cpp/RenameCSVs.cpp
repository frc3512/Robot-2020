// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "RenameCSVs.hpp"

#include <cstdio>
#include <experimental/filesystem>
#include <string>

void RenameCSVs(std::string_view prefix, std::string_view name) {
    for (auto& p : std::experimental::filesystem::directory_iterator(".")) {
        std::string directoryEntry = p.path().u8string();
        std::string oldFilename =
            directoryEntry.substr(directoryEntry.find("/") + 1);
        if (directoryEntry.find(name) == 0) {
            std::string newFilename =
                std::string{prefix.data()} + " " + oldFilename;
            std::rename(oldFilename.c_str(), newFilename.c_str());
        }
    }
}
