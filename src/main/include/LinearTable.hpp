// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

#ifndef ROBOT_2020_SRC_MAIN_INCLUDE_LINEARTABLE_HPP_
#define ROBOT_2020_SRC_MAIN_INCLUDE_LINEARTABLE_HPP_

#include <functional>
#include <memory>
#include <utility>

#include "LookupDetail.hpp"

namespace lookup {

template <typename Key, typename Value, typename Compare = std::less<Key>,
          typename Allocator = std::allocator<std::pair<const Key, Value> > >
class unbounded_linear_table
    : public detail::basic_lookup_table<Key, Value, Compare, Allocator> {
private:
    typedef detail::basic_lookup_table<Key, Value, Compare, Allocator> base;

public:
    typedef typename base::iterator iterator;
    typedef typename base::const_iterator const_iterator;
    typedef typename base::size_type size_type;
    typedef typename base::allocator allocator;
    typedef typename base::key_type key_type;
    typedef typename base::mapped_type mapped_type;
    typedef typename base::value_type value_type;
    typedef typename base::key_compare key_compare;
    typedef typename base::reference reference;
    typedef typename base::const_reference const_reference;
    typedef typename base::pointer pointer;
    typedef typename base::const_pointer const_pointer;

    // Returns an unbounded linear interpolation based on key.
    // Unbounded -> if the key is less than the minimum key in
    // the map, it will return the minimum value, if it is greater
    // than the maximum, it will return the maximum.

    mapped_type linear_interp(const Key& k) const {
        // First, test to see if the exact key
        // is actually in the table.
        const_iterator find = base::find(k);

        if (find != base::end()) {
            return find->second;
        }

        const_iterator higher = base::upper_bound(k);

        // Lower constraint; upper_bound is the
        // min table value
        if (higher == base::begin()) {
            return higher->second;
        }

        // Higher constraint check; upper bound (may)
        // be greater than max table value.
        if (higher == base::end()) {
            const_iterator end_iter = base::end();
            --end_iter;
            return end_iter->second;
        }

        const_iterator lower = higher;
        --lower;

        key_type diff_low = k - lower->first;
        key_type total = higher->first - lower->first;

        // Linearlly interpolate between lower and higher values
        return lower->second +
               (diff_low / total) * (higher->second - lower->second);
    }
};  // end class unbounded_linear_table

typedef unbounded_linear_table<double, double> unbounded_lookup1d;

}  // namespace lookup

#endif  // ROBOT_2020_SRC_MAIN_INCLUDE_LINEARTABLE_HPP_
