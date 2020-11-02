// Copyright (c) 2020-2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <rev/CANSparkMax.h>

namespace frc3512 {

enum class Usage { kAll, kPositionOnly, kVelocityOnly, kMinimal };

/**
 * This function allows reducing a Spark Max's CAN bus utilization by reducing
 * the periodic status frame period of nonessential frames from 20ms to 500ms.
 *
 * See https://www.revrobotics.com/sparkmax-users-manual/#section-3-3-2-1 for a
 * description of the status frames.
 *
 * @param motor       The motor to adjust the status frame periods on.
 * @param utilization The status frame feedack to enable. kAll is the default
 *                    when a CANSparkMax is constructed.
 */
void SetCANSparkMaxBusUsage(rev::CANSparkMax& motor, Usage usage);

}  // namespace frc3512
