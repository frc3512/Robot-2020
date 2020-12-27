// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

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

namespace frc3512 {

class SetCurrentPath {
public:
    /**
     * Creates a new directory and sets that as the current path.
     *
     * On destruction, the original current path is restored.
     */
    explicit SetCurrentPath(const std::filesystem::path& p)
        : m_originalPath{std::filesystem::current_path()} {
        std::filesystem::create_directories(p);
        std::filesystem::current_path(p);
    }

    ~SetCurrentPath() { std::filesystem::current_path(m_originalPath); }

private:
    std::filesystem::path m_originalPath;
};

}  // namespace frc3512
