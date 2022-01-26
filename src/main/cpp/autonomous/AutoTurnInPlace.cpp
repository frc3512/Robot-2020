// Copyright (c) FRC Team 3512. All Rights Reserved.

#include <frc/trajectory/constraint/MaxVelocityConstraint.h>
#include <frc/trajectory/constraint/RectangularRegionConstraint.h>
#include <wpi/numbers>

#include "Robot.hpp"

namespace frc3512 {

void Robot::AutoTurnInPlace() {
    // Make is visible on Field2D for testing purposes
    frc::Pose2d kInitialPose{12_m, 5.662_m, units::radian_t{0}};

    drivetrain.Reset(kInitialPose);

    drivetrain.TurnDrivetrainInPlace(units::radian_t{wpi::numbers::pi / 2});

    if (!m_autonChooser.Suspend([=] { return drivetrain.AtHeading(); })) {
        return;
    }
}
}  // namespace frc3512
