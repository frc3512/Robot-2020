// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

namespace frc3512 {

// See docs/system-architecture.md#Background_on_real-time_scheduling for an
// introduction to real-time scheduling.

// Give logging thread lower priority than main robot thread and autonomous
// thread to let them preempt logging thread
constexpr int kPrioLoggingThread = 14;

// Give main robot thread and autonomous thread the same priority so they won't
// preempt each other
constexpr int kPrioMainRobotThread = 15;
constexpr int kPrioAutonomousThread = 15;

// Give NI's SPI driver RT priority 33
constexpr int kPrioSPIGyro = 33;

// Give FRC_NetCommDaemon higher priority so it's not preempted by user code
// during high CPU utilization
constexpr int kPrioNetCommDaemon = 35;

// Give HAL Notifier thread highest priority so robot code Notifiers are
// accurately scheduled
constexpr int kPrioHALNotifierThread = 40;

static_assert(kPrioLoggingThread < kPrioMainRobotThread,
              "Main robot thread must preempt logging thread");
static_assert(
    kPrioMainRobotThread == kPrioAutonomousThread,
    "Main robot thread and autonomous thread must not preempt each other");
static_assert(kPrioMainRobotThread < kPrioSPIGyro,
              "NI SPI driver must preempt main robot thread");
static_assert(kPrioMainRobotThread < kPrioNetCommDaemon,
              "FRC_NetCommDaemon must preempt main robot thread");
static_assert(kPrioMainRobotThread < kPrioHALNotifierThread,
              "HAL Notifier thread must preempt main robot thread");

}  // namespace frc3512
