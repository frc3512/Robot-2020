// Copyright (c) FRC Team 3512. All Rights Reserved.

#include "CANSparkMaxUtil.hpp"

namespace frc3512 {

void SetCANSparkMaxBusUsage(rev::CANSparkMax& motor, Usage usage,
                            bool enableFollowing) {
    if (enableFollowing) {
        motor.SetPeriodicFramePeriod(rev::CANSparkMax::PeriodicFrame::kStatus0,
                                     10);
    } else {
        motor.SetPeriodicFramePeriod(rev::CANSparkMax::PeriodicFrame::kStatus0,
                                     500);
    }

    if (usage == Usage::kAll) {
        motor.SetPeriodicFramePeriod(rev::CANSparkMax::PeriodicFrame::kStatus1,
                                     20);
        motor.SetPeriodicFramePeriod(rev::CANSparkMax::PeriodicFrame::kStatus2,
                                     20);
        motor.SetPeriodicFramePeriod(rev::CANSparkMax::PeriodicFrame::kStatus3,
                                     50);
    } else if (usage == Usage::kPositionOnly) {
        motor.SetPeriodicFramePeriod(rev::CANSparkMax::PeriodicFrame::kStatus1,
                                     500);
        motor.SetPeriodicFramePeriod(rev::CANSparkMax::PeriodicFrame::kStatus2,
                                     20);
        motor.SetPeriodicFramePeriod(rev::CANSparkMax::PeriodicFrame::kStatus3,
                                     500);
    } else if (usage == Usage::kVelocityOnly) {
        motor.SetPeriodicFramePeriod(rev::CANSparkMax::PeriodicFrame::kStatus1,
                                     20);
        motor.SetPeriodicFramePeriod(rev::CANSparkMax::PeriodicFrame::kStatus2,
                                     500);
        motor.SetPeriodicFramePeriod(rev::CANSparkMax::PeriodicFrame::kStatus3,
                                     500);
    } else if (usage == Usage::kMinimal) {
        motor.SetPeriodicFramePeriod(rev::CANSparkMax::PeriodicFrame::kStatus1,
                                     500);
        motor.SetPeriodicFramePeriod(rev::CANSparkMax::PeriodicFrame::kStatus2,
                                     500);
        motor.SetPeriodicFramePeriod(rev::CANSparkMax::PeriodicFrame::kStatus3,
                                     500);
    }
}

}  // namespace frc3512
