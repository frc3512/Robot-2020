// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <thread>

#include <gtest/gtest.h>

#include "subsystems/ControllerSubsystemBase.hpp"
#include "subsystems/Drivetrain.hpp"
#include "subsystems/Flywheel.hpp"
#include "subsystems/Turret.hpp"

class MockSubsystem : public frc3512::ControllerSubsystemBase {
public:
    int periodicCount = 0;

    MockSubsystem() : ControllerSubsystemBase("Mock") {}

    void ControllerPeriodic() override { periodicCount++; }
};

TEST(ControllerSubsystemBase, ControllerPeriodic) {
    MockSubsystem subsystem;

    // Enable at 0ms
    frc3512::ControllerSubsystemBase::Enable();

    // ControllerPeriodic() will run at 5ms and 10ms timestamps
    std::this_thread::sleep_for(std::chrono::milliseconds(12));

    // Disable before reaching 15ms
    frc3512::ControllerSubsystemBase::Disable();

    EXPECT_EQ(subsystem.periodicCount, 2);
}

TEST(ControllerSubsystemBase, EnableController) {
    frc3512::Drivetrain drivetrain;
    frc3512::Turret turret{drivetrain};
    frc3512::Flywheel flywheel{turret};

    drivetrain.SetWaypoints(frc::Pose2d(0_m, 0_m, 0_rad), {},
                            frc::Pose2d(2_m, 1_m, 0_rad));

    frc3512::ControllerSubsystemBase::Enable();
    drivetrain.EnableController();
    turret.EnableController();
    flywheel.EnableController();
    frc3512::ControllerSubsystemBase::Disable();
}
