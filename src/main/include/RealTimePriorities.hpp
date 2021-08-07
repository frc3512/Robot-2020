// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#pragma once

namespace frc3512 {

// Priorities range from 1 to 99 with higher numbers being higher priority. We
// use the first-in first-out scheduler. From "man 7 sched" with terminal width
// of 87:
//
// SCHED_FIFO can be used only with static priorities higher than 0, which means
// that  when  a  SCHED_FIFO thread becomes runnable, it will always immediately
// preempt any currently running SCHED_OTHER, SCHED_BATCH, or SCHED_IDLE thread.
// SCHED_FIFO  is  a  simple  scheduling  algorithm  without  time slicing.  For
// threads scheduled under the SCHED_FIFO policy, the following rules apply:
//
// 1) A running SCHED_FIFO thread that has been preempted by another  thread  of
//    higher  priority  will  stay  at the head of the list for its priority and
//    will resume execution as soon  as  all  threads  of  higher  priority  are
//    blocked again.
//
// 2) When  a blocked SCHED_FIFO thread becomes runnable, it will be inserted at
//    the end of the list for its priority.
//
// 3) If a call to sched_setscheduler(2),  sched_setparam(2),  sched_setattr(2),
//    pthread_setschedparam(3),  or pthread_setschedprio(3) changes the priority
//    of the running or runnable SCHED_FIFO thread identified by pid the  effect
//    on  the  thread's  position  in  the  list depends on the direction of the
//    change to threads priority:
//
//    •  If the thread's priority is raised, it is placed at the end of the list
//       for  its  new  priority.   As a consequence, it may preempt a currently
//       running thread with the same priority.
//
//    •  If the thread's priority is unchanged, its position in the run list  is
//       unchanged.
//
//    •  If  the  thread's priority is lowered, it is placed at the front of the
//       list for its new priority.
//
//    According to POSIX.1-2008, changes to a thread's priority (or policy)  us‐
//    ing  any mechanism other than pthread_setschedprio(3) should result in the
//    thread being placed at the end of the list for its priority.
//
// 4) A thread calling sched_yield(2) will be put at the end of the list.
//
// No other events will move a thread scheduled under the SCHED_FIFO  policy  in
// the wait list of runnable threads with equal static priority.
//
// A  SCHED_FIFO thread runs until either it is blocked by an I/O request, it is
// preempted by a higher priority thread, or it calls sched_yield(2).

// Give logging thread lower priority than main robot thread and autonomous
// thread to let them preempt logging thread
constexpr int kPrioLoggingThread = 14;

// Give main robot thread and autonomous thread the same priority so they won't
// preempt each other
constexpr int kPrioMainRobotThread = 15;
constexpr int kPrioAutonomousThread = 15;

// Give NI's SPI driver RT priority 33
constexpr int kPrioSPIGyro = 33;

// Give FRC_NetCommDaemon higher priority so it's not preempted by robot code
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
