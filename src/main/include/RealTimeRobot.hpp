// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <functional>

#include <frc/TimedRobot.h>
#include <units/time.h>

namespace frc3512 {

/**
 * RealTimeRobot extends the TimedRobot robot program framework to offer better
 * real-time performance.
 *
 * The RealTimeRobot class is intended to be subclassed by a user creating a
 * robot program.
 *
 * Periodic() functions from the base class are called on an interval by a
 * Notifier instance.
 */
class RealTimeRobot : public frc::TimedRobot {
public:
    static constexpr auto kDefaultPriority = 15;
    static constexpr auto kDefaultControllerPeriod = 5_ms;

    /**
     * Constructor for RealTimeRobot.
     *
     * @param robotAllocation  The allocation to give the RealTimeRobot periodic
     *                         functions.
     * @param controllerPeriod The controller period. This is the basis for
     * @param priority         Priority to set the thread to. For real-time,
     *                         this is 1-99 with 99 being highest. For
     *                         non-real-time, this is forced to 0. See "man 7
     *                         sched" for more details.
     */
    explicit RealTimeRobot(
        units::second_t robotPeriodicAllocation = 2_ms,
        units::second_t controllerPeriod = kDefaultControllerPeriod,
        int priority = kDefaultPriority);

    /**
     * Schedule a periodic function with the constructor's controller period and
     * the given allocation. The function's runtime allocation will be placed
     * after the end of the previous one's.
     *
     * If a call to this function makes the allocations exceed the controller
     * period, an exception will be thrown since that means the RealTimeRobot
     * periodic functions and the given function will have conflicting
     * timeslices.
     *
     * @param func       Function to schedule.
     * @param allocation The function's runtime allocation out of the controller
     *                   period.
     */
    void Schedule(std::function<void()> func, units::second_t allocation);

private:
    units::second_t m_controllerPeriod;
    units::second_t m_nextOffset;
};

}  // namespace frc3512
