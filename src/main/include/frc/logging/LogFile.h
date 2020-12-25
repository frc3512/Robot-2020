/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <ctime>
#include <fstream>
#include <string>

#include <units/time.h>
#include <wpi/StringRef.h>

namespace frc {

/**
 * A LogFile writes text to log in a file.
 */
class LogFile {
 public:
  /**
   * Instantiate a LogFile passing in its prefix and its extension.
   *
   * If you want the file to be saved in a existing directory, you can add
   * its path before the file prefix. Exemple : to save the file in a usb stick
   * on the roborio ("/media/sda1/") : LogFile("/media/sda1/log").
   *
   * @param filePrefix    The prefix of the LogFile.
   * @param fileExtension The extension of the LogFile (without dot).
   */
  explicit LogFile(wpi::StringRef filePrefix = "log",
                   wpi::StringRef fileExtension = "txt");

  LogFile(LogFile&&) = default;
  LogFile& operator=(LogFile&&) = default;

  /**
   * Write text in the LogFile.
   *
   * @param text The text to be logged in the file.
   */
  void Log(const wpi::StringRef& text);

  /**
   * Flushes lines written to the log file to disk.
   */
  void Flush();

  template <typename Value>
  friend LogFile& operator<<(LogFile& file, const Value& value) {
    file.m_file << value;
    file.UpdateFilename();
    return file;
  }

 private:
  /**
   * Check if the time has changed of more than 24 hours. Change the filename if
   * the condition is met.
   */
  void UpdateFilename();

  /**
   * Create a filename with a time.
   *
   * @param time The time that is saved in the filename.
   * @return The filename with the format "{filePrefix}-{date/time}.txt".
   */
  std::string CreateFilename(std::time_t time) const;

  std::string m_filePrefix;
  std::string m_fileExtension;
  std::time_t m_time;
  std::string m_filename;
  std::ofstream m_file;
};

}  // namespace frc
