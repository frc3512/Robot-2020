// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#pragma once

#include <cstddef>

#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <wpi/static_circular_buffer.h>

namespace frc3512 {

/**
 * Simulates balls in the intake subsystem.
 */
class IntakeSim {
public:
    /**
     * Add ball to the front of the intake.
     */
    void AddBall();

    /**
     * Returns the number of balls in the intake.
     */
    size_t NumberOfBalls() const;

    /**
     * Remove all balls from intake.
     */
    void RemoveAllBalls();

    /**
     * Runs intake simulation for the given timestep.
     *
     * @param runConveyor Whether balls should be moving.
     * @param dt          Timestep.
     * @return True if a ball reached the shooter.
     */
    bool Update(bool runConveyor, units::second_t dt);

private:
    struct Ball {
        units::meter_t position = 0_m;
        units::meters_per_second_t velocity = 0_mps;
    };

    wpi::static_circular_buffer<Ball, 5> m_balls;
};

}  // namespace frc3512
