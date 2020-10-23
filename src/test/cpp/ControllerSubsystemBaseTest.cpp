// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include <frc/simulation/SimHooks.h>
#include <gtest/gtest.h>

#include "Constants.hpp"
#include "subsystems/ControllerSubsystemBase.hpp"

namespace {
class ControllerSubsystemBaseTest : public testing::Test {
protected:
    void SetUp() override { frc::sim::PauseTiming(); }

    void TearDown() override { frc::sim::ResumeTiming(); }
};

class MockSubsystem : public frc3512::ControllerSubsystemBase {
public:
    int periodicCount = 0;

    void ControllerPeriodic() override { periodicCount++; }
};
}  // namespace

TEST_F(ControllerSubsystemBaseTest, ControllerPeriodic1) {
    MockSubsystem subsystem;

    // Enable at 0ms
    frc3512::ControllerSubsystemBase::Enable();

    // ControllerPeriodic() will run at 5ms and 10ms timestamps
    frc::sim::StepTiming(2 * frc3512::Constants::kDt);

    // Disable before reaching 15ms
    frc3512::ControllerSubsystemBase::Disable();

    EXPECT_EQ(subsystem.periodicCount, 2);
}

TEST_F(ControllerSubsystemBaseTest, ControllerPeriodic2) {
    MockSubsystem subsystem1;
    MockSubsystem subsystem2;

    // Enable at 0ms
    frc3512::ControllerSubsystemBase::Enable();

    // ControllerPeriodic() will run at 5ms and 10ms timestamps
    frc::sim::StepTiming(2 * frc3512::Constants::kDt);

    // Disable before reaching 15ms
    frc3512::ControllerSubsystemBase::Disable();

    EXPECT_EQ(subsystem1.periodicCount, 2);
    EXPECT_EQ(subsystem2.periodicCount, 2);
}
