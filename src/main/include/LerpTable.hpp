// Copyright (c) FRC Team 3512. All Rights Reserved.

#pragma once

#include <map>
#include <utility>

namespace frc3512 {

/**
 * Implements a table of key-value pairs with linear interpolation between
 * values.
 *
 * If there's no matching key, the value returned will be a linear interpolation
 * between the keys before and after the provided one.
 *
 * @tparam Key   The key type.
 * @tparam Value The value type.
 */
template <typename Key, typename Value>
class LerpTable {
public:
    /**
     * Inserts a key-value pair.
     *
     * @param key   The key.
     * @param value The value.
     */
    void Insert(const Key& key, const Value& value) {
        m_container.insert(std::make_pair(key, value));
    }

    /**
     * Inserts a key-value pair.
     *
     * @param key   The key.
     * @param value The value.
     */
    void Insert(Key&& key, Value&& value) {
        m_container.insert(std::make_pair(key, value));
    }

    /**
     * Returns the value associated with a given key.
     *
     * If there's no matching key, the value returned will be a linear
     * interpolation between the keys before and after the provided one.
     *
     * @param key The key.
     */
    Value operator[](const Key& key) const {
        using const_iterator = typename std::map<Key, Value>::const_iterator;

        // Get iterator to upper bound key-value pair for the given key
        const_iterator upper = m_container.upper_bound(key);

        // If key > largest key in table, return value for largest table key
        if (upper == m_container.end()) {
            return (--upper)->second;
        }

        // If key <= smallest key in table, return value for smallest table key
        if (upper == m_container.begin()) {
            return upper->second;
        }

        // Get iterator to lower bound key-value pair
        const_iterator lower = upper;
        --lower;

        // Perform linear interpolation between lower and upper bound
        const double delta =
            (key - lower->first) / (upper->first - lower->first);
        return delta * upper->second + (1.0 - delta) * lower->second;
    }

    /**
     * Clears the contents.
     */
    void Clear() { m_container.clear(); }

private:
    std::map<Key, Value> m_container;
};

}  // namespace frc3512
