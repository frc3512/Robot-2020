// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <frc/logging/CSVLogFile.h>
#include <frc2/Timer.h>
#include <wpi/SmallString.h>
#include <wpi/Twine.h>

namespace frc3512 {

/**
 * Member variable:
 * ScheduleLogger m_logger;
 *
 * Put the following at the start of the code to log.
 * auto context = m_logger.GetSchedContext();
 *
 * It logs the runtime automatically when it goes out of scope.
 */
class ScheduleLogger {
public:
    class SchedContext {
    public:
        SchedContext(ScheduleLogger& logger, const wpi::Twine& name)
            : m_logger{&logger}, m_name{name.str()} {}

        ~SchedContext() { Destroy(); }

        void Destroy() {
            if (m_valid) {
                m_logger->m_file.Log(
                    m_logger->m_epochTime, m_startTime.to<double>(),
                    (frc2::Timer::GetFPGATimestamp() - m_startTime)
                        .to<double>(),
                    m_name);
                m_valid = false;
            }
        }

    private:
        ScheduleLogger* m_logger;
        wpi::SmallString<128> m_name;
        units::second_t m_startTime = frc2::Timer::GetFPGATimestamp();

        bool m_valid = true;
    };

    void Epoch() {
        m_epochTime =
            frc2::Timer::GetFPGATimestamp() - frc::CSVLogFile::GetStartTime();
    }

    SchedContext GetSchedContext(const wpi::Twine& name) {
        return SchedContext{*this, name};
    }

private:
    frc::CSVLogFile m_file{"Schedule", "Start time (s)", "Duration (s)",
                           "Name"};
    units::second_t m_epochTime;
};

}  // namespace frc3512
