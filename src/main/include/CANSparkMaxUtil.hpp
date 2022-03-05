// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <rev/CANSparkMax.h>

namespace frc3512 {

enum class Usage { kAll, kPositionOnly, kVelocityOnly, kMinimal };

/**
 * This function allows reducing a Spark Max's CAN bus utilization by reducing
 * the periodic status frame period of nonessential frames from 20ms to 500ms.
 *
 * See
 * https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
 * for a description of the status frames.
 *
 * @param motor The motor to adjust the status frame periods on.
 * @param usage The status frame feedack to enable. kAll is the default
 *              when a CANSparkMax is constructed.
 * @param enableFollowing Whether to enable motor following.
 */
void SetCANSparkMaxBusUsage(rev::CANSparkMax& motor, Usage usage,
                            bool enableFollowing = false);

}  // namespace frc3512
