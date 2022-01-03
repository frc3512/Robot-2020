// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <cstddef>
#include <optional>
#include <utility>

#include <wpi/mutex.h>
#include <wpi/static_circular_buffer.h>

namespace frc3512 {

/**
 * Implements a thread-safe FIFO queue with a circular buffer as the storage.
 */
template <typename T, size_t N>
class static_concurrent_queue {
public:
    /**
     * Returns true if the queue is empty.
     */
    bool empty() const {
        std::scoped_lock lock(m_mutex);
        return m_queue.size() > 0;
    }

    /**
     * Returns the number of elements in the queue.
     */
    size_t size() const {
        std::scoped_lock lock(m_mutex);
        return m_queue.size();
    }

    /**
     * Returns the element at the front of the queue or std::nullopt_t if the
     * queue was empty.
     */
    std::optional<T> pop() {
        std::unique_lock lock(m_mutex);
        if (m_queue.size() > 0) {
            return m_queue.pop_back();
        } else {
            return std::nullopt;
        }
    }

    /**
     * Pushes an element into the back of the queue.
     *
     * @param item The element to push.
     */
    void push(const T& item) {
        std::scoped_lock lock(m_mutex);
        m_queue.push_front(item);
    }

    /**
     * Pushes an element into the back of the queue.
     *
     * @param item The element to push.
     */
    void push(T&& item) {
        std::scoped_lock lock(m_mutex);
        m_queue.push_front(std::forward<T>(item));
    }

    /**
     * Constructs an element with the given arguments and pushes it into the
     * back of the queue.
     *
     * @param args The arguments to use when constructing the element.
     */
    template <typename... Args>
    void emplace(Args&&... args) {
        std::scoped_lock lock(m_mutex);
        m_queue.emplace_front(std::forward<Args>(args)...);
    }

    /**
     * Clears the queue.
     */
    void reset() {
        std::scoped_lock lock(m_mutex);
        m_queue.reset();
    }

    static_concurrent_queue() = default;

    /**
     * Move constructor.
     */
    static_concurrent_queue(static_concurrent_queue&& rhs) {
        std::swap(m_queue, rhs.m_queue);
    }

    /**
     * Move assignment operator.
     */
    static_concurrent_queue& operator=(static_concurrent_queue&& rhs) {
        std::swap(m_queue, rhs.m_queue);
        return *this;
    }

private:
    wpi::static_circular_buffer<T, N> m_queue;
    mutable wpi::mutex m_mutex;
};

}  // namespace frc3512
