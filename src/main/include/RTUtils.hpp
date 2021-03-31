// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <string_view>

namespace frc3512 {

/**
 * Gives a currently running process the specified RT priority.
 *
 * @param process  A substring of the process name.
 * @param priority Priority to set the thread to. For real-time, this is 1-99
 *                 with 99 being highest. For non-real-time, this is forced to
 *                 0. See "man 7 sched" for more details.
 * @throw std::runtime_exception Runtime exception if setting RT priority
 *                               failed.
 */
void SetProcessRTPriority(std::string_view process, int priority);

}  // namespace frc3512
