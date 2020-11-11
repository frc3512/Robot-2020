/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/logging/LogFile.h"

#include <fmt/chrono.h>
#include <fmt/format.h>
#include <wpi/FileSystem.h>
#include <wpi/SmallString.h>

#include "frc/Filesystem.h"

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
  wpi::SmallString<64> path;
  frc::filesystem::GetOperatingDirectory(path);

  return fmt::format("{}/{}-{:%Y-%m-%d-%H_%M_%S}.{}",
                     std::string_view{path.str().data(), path.str().size()},
                     m_filePrefix, fmt::localtime(time), m_fileExtension);
}

void LogFile::Flush() { m_file.flush(); }
