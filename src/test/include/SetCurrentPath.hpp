// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#if __has_include(<filesystem>)
#include <filesystem>

#elif __has_include(<experimental/filesystem>)
#include <experimental/filesystem>
namespace std {
namespace filesystem = experimental::filesystem;
}  // namespace std
#endif

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

    SetCurrentPath(SetCurrentPath&&) = default;
    SetCurrentPath& operator=(SetCurrentPath&&) = default;

private:
    std::filesystem::path m_originalPath;
};

}  // namespace frc3512
