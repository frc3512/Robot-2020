// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#pragma once

#include <cstddef>
#include <optional>
#include <utility>

#include <wpi/mutex.h>
#include <wpi/static_circular_buffer.h>

namespace frc3512 {

template <typename T, size_t N>
class static_concurrent_queue {
public:
    bool empty() const {
        std::scoped_lock lock(m_mutex);
        return m_queue.size() > 0;
    }

    size_t size() const {
        std::scoped_lock lock(m_mutex);
        return m_queue.size();
    }

    std::optional<T> pop() {
        std::unique_lock lock(m_mutex);
        if (m_queue.size() > 0) {
            return m_queue.pop_back();
        } else {
            return std::nullopt;
        }
    }

    void push(const T& item) {
        std::scoped_lock lock(m_mutex);
        m_queue.push_front(item);
    }

    void push(T&& item) {
        std::scoped_lock lock(m_mutex);
        m_queue.push_front(std::forward<T>(item));
    }

    template <typename... Args>
    void emplace(Args&&... args) {
        std::scoped_lock lock(m_mutex);
        m_queue.emplace_front(std::forward<Args>(args)...);
    }

    void reset() {
        std::scoped_lock lock(m_mutex);
        m_queue.reset();
    }

    static_concurrent_queue() = default;
    static_concurrent_queue(const static_concurrent_queue&) = delete;
    static_concurrent_queue& operator=(const static_concurrent_queue&) = delete;

private:
    wpi::static_circular_buffer<T, N> m_queue;
    mutable wpi::mutex m_mutex;
};

}  // namespace frc3512
