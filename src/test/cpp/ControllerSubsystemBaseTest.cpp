// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <thread>

#include <gtest/gtest.h>

#include "subsystems/ControllerSubsystemBase.hpp"

class MockSubsystem : public frc3512::ControllerSubsystemBase {
public:
    int periodicCount = 0;

    MockSubsystem() : ControllerSubsystemBase("Mock") {}

    void ControllerPeriodic() override { periodicCount++; }
};

TEST(ControllerSubsystemBase, ControllerPeriodic1) {
    MockSubsystem subsystem;

    // Enable at 0ms
    frc3512::ControllerSubsystemBase::Enable();

    // ControllerPeriodic() will run at 5ms and 10ms timestamps
    std::this_thread::sleep_for(std::chrono::milliseconds(12));

    // Disable before reaching 15ms
    frc3512::ControllerSubsystemBase::Disable();

    EXPECT_EQ(subsystem.periodicCount, 2);
}

TEST(ControllerSubsystemBase, ControllerPeriodic2) {
    MockSubsystem subsystem1;
    MockSubsystem subsystem2;

    // Enable at 0ms
    frc3512::ControllerSubsystemBase::Enable();

    // ControllerPeriodic() will run at 5ms and 10ms timestamps
    std::this_thread::sleep_for(std::chrono::milliseconds(12));

    // Disable before reaching 15ms
    frc3512::ControllerSubsystemBase::Disable();

    EXPECT_EQ(subsystem1.periodicCount, 2);
    EXPECT_EQ(subsystem2.periodicCount, 2);
}
