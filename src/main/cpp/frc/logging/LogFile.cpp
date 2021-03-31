// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "frc/logging/LogFile.h"

#include <vector>

#include <fmt/chrono.h>
#include <fmt/format.h>
#include <wpi/FileSystem.h>
#include <wpi/SmallString.h>

#include "frc/Filesystem.h"
#include "frc/RobotBase.h"

using namespace frc;

LogFile::LogFile(wpi::StringRef filePrefix, wpi::StringRef fileExtension)
    : m_filePrefix{filePrefix},
      m_fileExtension{fileExtension},
      m_time{std::time(nullptr)},
      m_filename{CreateFilename(m_time)},
      m_file{m_filename} {
  if (m_file.fail()) {
    fmt::print(stderr, "Could not open file `{}` for writing.\n", m_filename);
    return;
  }
}

void LogFile::Log(const wpi::StringRef& text) {
  m_file << text;
  UpdateFilename();
}

void LogFile::UpdateFilename() {
  std::time_t newTime = std::time(nullptr);

  // If the difference between the two timestamps is too long
  if (units::second_t{std::difftime(newTime, m_time)} > 1_d) {
    std::string newName = CreateFilename(newTime);
    m_file.close();
    std::rename(CreateFilename(m_time).c_str(), newName.c_str());
    m_file.open(newName);
  }

  m_time = newTime;
}

std::string LogFile::CreateFilename(std::time_t time) const {
  // Write to USB storage when running on the roboRIO
  if constexpr (RobotBase::IsSimulation()) {
    wpi::SmallString<64> path;
    frc::filesystem::GetOperatingDirectory(path);
    return fmt::format("{}/{}-{:%Y-%m-%d-%H_%M_%S}.{}",
                       std::string_view{path.str().data(), path.str().size()},
                       m_filePrefix, fmt::localtime(time), m_fileExtension);
  } else {
    using wpi::sys::fs::directory_iterator;

    std::vector<std::string> files;
    std::error_code ec;
    for (auto p = directory_iterator("/media", ec, false);
         p != directory_iterator(); p.increment(ec)) {
      if (ec) {
        break;
      }
      files.emplace_back(p->path());
    }

    // If there's storage mounted in /media, put log files in the first
    // directory found
    if (files.size() > 0) {
      return fmt::format("{}/{}-{:%Y-%m-%d-%H_%M_%S}.{}", files[0],
                         m_filePrefix, fmt::localtime(time), m_fileExtension);
    } else {
      wpi::SmallString<64> path;
      frc::filesystem::GetOperatingDirectory(path);
      return fmt::format("{}/{}-{:%Y-%m-%d-%H_%M_%S}.{}",
                         std::string_view{path.str().data(), path.str().size()},
                         m_filePrefix, fmt::localtime(time), m_fileExtension);
    }
  }
}

void LogFile::Flush() { m_file.flush(); }
