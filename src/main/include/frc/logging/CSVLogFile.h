// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <algorithm>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>

#include <units/time.h>

#include "frc/Timer.h"
#include "frc/logging/LogFile.h"

namespace frc {

/**
 * A CSVLogFile writes values to a csv file
 *
 * For the CSVLogFile to write log informations, you must call Log()
 * periodically.
 */
class CSVLogFile {
 public:
  /**
   * Instantiate a LogFile passing in its prefix and its column headings.
   *
   * If you want the file to be saved in a existing directory, you can add
   * its path before the file prefix. Exemple : to save the file in a usb stick
   * on the roborio ("/media/sda1/") : LogFile("/media/sda1/log").
   *
   * @param filePrefix     The prefix of the LogFile.
   * @param columnHeading  Title of 1st CSVLogFile column.
   * @param columnHeadings Titles of other CSVLogFile columns.
   */
  template <typename Value, typename... Values>
  CSVLogFile(std::string_view filePrefix, Value columnHeading,
             Values... columnHeadings)
      : m_logFile(filePrefix, "csv") {
    m_logFile << "\"Time (s)\",";
    LogValues(columnHeading, columnHeadings...);
  }

  /**
   * Instantiate a LogFile passing in its prefix and its column headings.
   *
   * If you want the file to be saved in a existing directory, you can add
   * its path before the file prefix. Exemple : to save the file in a usb stick
   * on the roborio ("/media/sda1/") : LogFile("/media/sda1/log").
   *
   * @param filePrefix     The prefix of the LogFile.
   * @param columnHeadings Titles of CSVLogFile columns.
   */
  template <typename... Values>
  CSVLogFile(std::string_view filePrefix,
             const std::tuple<Values...>& columnHeadings)
      : m_logFile(filePrefix, "csv") {
    static_assert(sizeof...(Values) > 0,
                  "At least one column heading is required");
    m_logFile << "\"Time (s)\",";
    std::apply(&CSVLogFile::LogValues<Values...>,
               std::tuple_cat(std::tuple{this}, columnHeadings));
  }

  /**
   * Move constructor.
   */
  CSVLogFile(CSVLogFile&&) = default;

  /**
   * Move assignment operator.
   */
  CSVLogFile& operator=(CSVLogFile&&) = default;

  /**
   * Print a new line of values in the CSVLogFile.
   *
   * @param time   The timestamp for this line of values.
   * @param value  First value to log in the file.
   * @param values Other values to log in the file in order.
   */
  template <typename Value, typename... Values>
  void Log(units::second_t time, Value value, Values... values) {
    m_logFile << time.value() << ',';
    LogValues(value, values...);
  }

  /**
   * Print a new line of values in the CSVLogFile.
   *
   * @param value  First value to log in the file.
   * @param values Other values to log in the file in order.
   */
  template <typename Value, typename... Values>
  void Log(Value value, Values... values) {
    Log(frc::Timer::GetFPGATimestamp() - GetStartTime(), value, values...);
  }

  /**
   * Returns the timestamp when the robot program started.
   */
  static units::second_t GetStartTime() {
    static units::second_t startTime = frc::Timer::GetFPGATimestamp();
    return startTime;
  }

 private:
  /**
   * Print a new line of values in the CSVLogFile without timestamp.
   *
   * @param value  First value to log in the file.
   * @param values Other values to log in the file in order.
   */
  template <typename Value, typename... Values>
  void LogValues(Value value, Values... values) {
    if constexpr (std::is_convertible_v<Value, std::string_view>) {
      m_logFile << '\"' << EscapeDoubleQuotes(value) << '\"';
    } else {
      m_logFile << value;
    }

    if constexpr (sizeof...(values) > 0) {
      m_logFile << ',';
      LogValues(values...);
    } else {
      m_logFile << '\n';
      m_logFile.Flush();
    }
  }

  /**
   * Escape double quotes in a text by duplicating them.
   *
   * @param text Text to escape.
   * @return The text with all its double quotes escaped.
   */
  std::string EscapeDoubleQuotes(std::string_view text) const {
    std::string textString{text};
    for (std::string::size_type i = 0; i < text.size(); i++) {
      if (text[i] == '\"') {
        i++;
        textString.insert(i, "\"");
      }
    }
    return textString;
  }

  LogFile m_logFile;
};

}  // namespace frc
