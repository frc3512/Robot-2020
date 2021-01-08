// Copyright (c) 2021 FRC Team 3512. All Rights Reserved.

#include "IntakeSim.hpp"

using namespace frc3512;

void IntakeSim::AddBall() {
    // Move other balls out of the way first
    if (m_balls.size() > 0 && m_balls.size() < 3) {
        for (auto& ball : m_balls) {
            ball.position += 0.15_m;
        }
    }

    if (m_balls.size() == 3) {
        m_balls.emplace_back(0.55_m, 0_mps);
    } else if (m_balls.size() == 4) {
        m_balls.emplace_back(0.4_m, 0_mps);
    } else {
        m_balls.emplace_back(0.7_m, 0_mps);
    }
}

size_t IntakeSim::NumberOfBalls() const { return m_balls.size(); }

void IntakeSim::RemoveAllBalls() { m_balls.reset(); }

bool IntakeSim::Update(bool runConveyor, units::second_t dt) {
    bool ballAtShooter = false;

    size_t i = 0;
    while (i < m_balls.size()) {
        auto& ball = m_balls[i];

        if (runConveyor || ball.position > 1.1_m) {
            ball.velocity = 0.7_mps;
        } else {
            ball.velocity = 0_mps;
        }

        ball.position += ball.velocity * dt;

        if (ball.position > 1.1_m) {
            ballAtShooter = true;

            if (ball.position > 1.2_m) {
                m_balls.pop_front();
                continue;
            }
        }

        ++i;
    }

    return ballAtShooter;
}
