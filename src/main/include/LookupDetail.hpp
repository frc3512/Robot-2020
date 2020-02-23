// Copyright (c) 2020 FRC Team 3512. All Rights Reserved.

// Internal Header: Not to be directly imported

#ifndef ROBOT_2020_SRC_MAIN_INCLUDE_LOOKUPDETAIL_HPP_
#define ROBOT_2020_SRC_MAIN_INCLUDE_LOOKUPDETAIL_HPP_

#include <map>
#include <utility>

namespace lookup {
namespace detail {

template <typename Key, typename Value, typename Compare, typename Allocator>
class basic_lookup_table {
private:
    typedef std::map<Key, Value, Compare, Allocator> container;
    container table_;

public:
    typedef typename container::iterator iterator;
    typedef typename container::const_iterator const_iterator;
    typedef typename container::size_type size_type;
    typedef typename container::reference reference;
    typedef typename container::const_reference const_reference;
    typedef typename container::pointer pointer;
    typedef typename container::const_pointer const_pointer;
    typedef typename container::value_type value_type;
    typedef Allocator allocator;
    typedef Key key_type;
    typedef Value mapped_type;
    typedef Compare key_compare;

protected:
    key_compare cmp_;

    // Disallow polymorphic usage through derived pointer
    ~basic_lookup_table() {}

    iterator upper_bound(const Key& k) { return table_.upper_bound(k); }

    const_iterator upper_bound(const Key& k) const {
        return table_.upper_bound(k);
    }

    iterator lower_bound(const Key& k) { return table_.lower_bound(k); }

    const_iterator lower_bound(const Key& k) const {
        return table_.lower_bound(k);
    }

    iterator find(const Key& k) { return table_.find(k); }

    const_iterator find(const Key& k) const { return table_.find(k); }

public:
    void insert(const key_type& key, const mapped_type& value) {
        table_.insert(std::make_pair(key, value));
    }

#if __cplusplus >= 201103L

    void insert(key_type&& key, mapped_type&& value) {
        table_.insert(std::make_pair(key, value));
    }

#endif

    bool erase_key(const key_type& k) {
        size_type s = table_.erase(k);
        return s != 0;
    }

    void erase_greater(const key_type& k) {
        iterator bound = table_.upper_bound(k);
        table_.erase(bound, table_.end());
    }

    void erase_less(const key_type& k) {
        iterator bound = table_.lower_bound(k);
        table_.erase(table_.begin(), bound);
    }

    void clear() { table_.clear(); }

    iterator begin() { return table_.begin(); }

    const_iterator begin() const { return table_.begin(); }

    iterator end() { return table_.end(); }

    const_iterator end() const { return table_.end(); }
};

}  // namespace detail
}  // namespace lookup

#endif  // ROBOT_2020_SRC_MAIN_INCLUDE_LOOKUPDETAIL_HPP_
