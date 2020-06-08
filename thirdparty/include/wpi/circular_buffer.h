/*----------------------------------------------------------------------------*/
/* Copyright (c) 2015-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <array>
#include <cstddef>

namespace wpi {

/**
 * This is a simple circular buffer so we don't need to "bucket brigade" copy
 * old values.
 */
template <class T, size_t N>
class circular_buffer {
 public:
  static_assert(N > 0, "The circular buffer size shouldn't be zero.");

  constexpr circular_buffer() = default;

  class iterator {
   public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = T;
    using difference_type = std::ptrdiff_t;
    using pointer = T*;
    using reference = T&;

    explicit iterator(circular_buffer* buffer, size_t index)
        : m_buffer(buffer), m_index(index) {}

    iterator& operator++() {
      ++m_index;
      return *this;
    }
    iterator operator++(int) {
      iterator retval = *this;
      ++(*this);
      return retval;
    }
    bool operator==(iterator other) const {
      return m_buffer == other.m_buffer && m_index == other.m_index;
    }
    bool operator!=(iterator other) const { return !(*this == other); }
    reference operator*() const { return (*m_buffer)[m_index]; }

   private:
    circular_buffer* m_buffer;
    size_t m_index;
  };

  class const_iterator {
   public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = T;
    using difference_type = std::ptrdiff_t;
    using pointer = T*;
    using reference = T&;

    explicit const_iterator(const circular_buffer* buffer, size_t index)
        : m_buffer(buffer), m_index(index) {}

    const_iterator& operator++() {
      ++m_index;
      return *this;
    }
    const_iterator operator++(int) {
      const_iterator retval = *this;
      ++(*this);
      return retval;
    }
    bool operator==(const_iterator other) const {
      return m_buffer == other.m_buffer && m_index == other.m_index;
    }
    bool operator!=(const_iterator other) const { return !(*this == other); }
    const reference operator*() const { return (*m_buffer)[m_index]; }

   private:
    const circular_buffer* m_buffer;
    size_t m_index;
  };

  iterator begin() { return iterator(this, 0); }
  iterator end() {
    return iterator(this, ::wpi::circular_buffer<T, N>::size());
  }

  const_iterator begin() const { return const_iterator(this, 0); }
  const_iterator end() const {
    return const_iterator(this, ::wpi::circular_buffer<T, N>::size());
  }

  /**
   * Returns number of elements in buffer
   */
  size_t size() const { return m_length; }

  /**
   * Returns value at front of buffer
   */
  T& front() { return (*this)[0]; }

  /**
   * Returns value at front of buffer
   */
  const T& front() const { return (*this)[0]; }

  /**
   * Returns value at back of buffer
   *
   * If there are no elements in the buffer, calling this function results in
   * undefined behavior.
   */
  T& back() { return m_data[(m_front + m_length - 1) % N]; }

  /**
   * Returns value at back of buffer
   *
   * If there are no elements in the buffer, calling this function results in
   * undefined behavior.
   */
  const T& back() const { return m_data[(m_front + m_length - 1) % N]; }

  /**
   * Push new value onto front of the buffer. The value at the back is
   * overwritten if the buffer is full.
   */
  void push_front(T value) {
    m_front = ModuloDec(m_front);

    m_data[m_front] = value;

    if (m_length < N) {
      m_length++;
    }
  }

  /**
   * Push new value onto back of the buffer. The value at the front is
   * overwritten if the buffer is full.
   */
  void push_back(T value) {
    m_data[(m_front + m_length) % N] = value;

    if (m_length < N) {
      m_length++;
    } else {
      // Increment front if buffer is full to maintain size
      m_front = ModuloInc(m_front);
    }
  }

  /**
   * Pop value at front of buffer.
   *
   * If there are no elements in the buffer, calling this function results in
   * undefined behavior.
   */
  T pop_front() {
    T& temp = m_data[m_front];
    m_front = ModuloInc(m_front);
    m_length--;
    return temp;
  }

  /**
   * Pop value at back of buffer.
   *
   * If there are no elements in the buffer, calling this function results in
   * undefined behavior.
   */
  T pop_back() {
    m_length--;
    return m_data[(m_front + m_length) % N];
  }

  /**
   * Empties internal buffer.
   */
  void reset() {
    m_front = 0;
    m_length = 0;
  }

  /**
   * @return Element at index starting from front of buffer.
   */
  T& operator[](size_t index) { return m_data[(m_front + index) % N]; }

  /**
   * @return Element at index starting from front of buffer.
   */
  const T& operator[](size_t index) const {
    return m_data[(m_front + index) % N];
  }

 private:
  std::array<T, N> m_data;

  // Index of element at front of buffer
  size_t m_front = 0;

  // Number of elements used in buffer
  size_t m_length = 0;

  /**
   * Increment an index modulo the length of the buffer.
   *
   * @return The result of the modulo operation.
   */
  size_t ModuloInc(size_t index) { return (index + 1) % N; }

  /**
   * Decrement an index modulo the length of the buffer.
   *
   * @return The result of the modulo operation.
   */
  size_t ModuloDec(size_t index) {
    if (index == 0) {
      return N - 1;
    } else {
      return index - 1;
    }
  }
};

}  // namespace wpi
