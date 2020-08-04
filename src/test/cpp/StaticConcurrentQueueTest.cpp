// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#include "static_concurrent_queue.hpp"  // NOLINT(build/include_order)

#include <array>

#include <gtest/gtest.h>

static const std::array<double, 10> values = {
    {751.848, 766.366, 342.657, 234.252, 716.126, 132.344, 445.697, 22.727,
     421.125, 799.913}};

static const std::array<double, 8> pushFrontOut = {
    {799.913, 421.125, 22.727, 445.697, 132.344, 716.126, 234.252, 342.657}};

TEST(StaticConcurrentQueueTest, PushTest) {
    frc3512::static_concurrent_queue<double, 8> queue;

    for (auto& value : values) {
        queue.push(value);
    }

    for (size_t i = 0; i < pushFrontOut.size(); ++i) {
        EXPECT_EQ(pushFrontOut[pushFrontOut.size() - i - 1],
                  queue.pop().value());
    }
}

TEST(StaticConcurrentQueueTest, EmplaceTest) {
    frc3512::static_concurrent_queue<double, 8> queue;

    for (auto& value : values) {
        queue.emplace(value);
    }

    for (size_t i = 0; i < pushFrontOut.size(); ++i) {
        EXPECT_EQ(pushFrontOut[pushFrontOut.size() - i - 1],
                  queue.pop().value());
    }
}

TEST(StaticConcurrentQueueTest, ResetTest) {
    frc3512::static_concurrent_queue<double, 5> queue;

    for (size_t i = 1; i < 6; ++i) {
        queue.push(i);
    }

    queue.reset();

    EXPECT_EQ(queue.size(), size_t{0});
}
