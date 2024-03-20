#pragma once

#include <unordered_map>

/**
 * A TypeMap is an unordered hash map that uses types as its keys.
 *
 * See https://gpfault.net/posts/mapping-types-to-values.txt.html
 * for details.
 *
 * @tparam TValue the type of the values in the TypeMap
 */
template <class TValue>
class TypeMap
{
    // Internally, we'll use a hashmap to store mappings from type
    // IDs to values
    using InternalMap = std::unordered_map<int, TValue>;

   public:
    using Iterator      = typename InternalMap::iterator;
    using ConstIterator = typename InternalMap::const_iterator;

    explicit TypeMap();

    /**
     * Iterator methods
     *
     * @see begin and end for std::unordered_map
     */
    Iterator begin();
    Iterator end();
    ConstIterator begin() const;
    ConstIterator end() const;

    /**
     * Finds the element associated with the type `TKey` in the TypeMap
     *
     * @tparam TKey the type of the key to lookup in the TypeMap
     * @return the element associated with the given key type
     */
    template <class TKey>
    Iterator find();

    /**
     * Const version of find
     */
    template <class TKey>
    ConstIterator find() const;

    /**
     * Returns whether the key exists in the TypeMap
     * 
     * @tparam TKey the type of the key to lookup in the TypeMap
     * @return true if the key exists in the TypeMap, false otherwise
     */
    template <class TKey>
    bool contains() const;

    /**
     * Associates a value with the type `TKey` in the TypeMap
     *
     * @tparam TKey the type of the key to lookup in the TypeMap
     * @param value the value to associate with the key
     */
    template <class TKey>
    void put(TValue&& value);

    /**
     * Returns a reference to the value that is mapped to the type `TKey`,
     * performing an insertion if such key does not already exist
     *
     * @see std::unordered_map::operator[]
     *
     * @tparam TKey the type of the key to lookup in the TypeMap
     * @return a reference to the value mapped to the given key
     */
    template <class TKey>
    TValue& at();

    /**
     * Erases all elements from the TypeMap.
     */
    void clear();

   private:
    InternalMap map_;
    static int type_id_counter_;

    /**
     * Obtains a unique identifier for the given `TKey` type
     *
     * @return a unique ID for the given `TKey` type
     */
    template <class TKey>
    inline static int getTypeId()
    {
        // It is not guaranteed that std::type_info::hash_code will return
        // different hash codes for different types, so we cannot rely on it.

        // The trick here is that every template instantiation of getTypeId
        // is considered a completely different function, so they each will
        // have completely different static variables `id` instantiated to
        // different values. Here, `id` is instantiated to whatever the global
        // type id counter was at the time of the template instantiation.
        static const int id = type_id_counter_++;
        return id;
    }
};
#include "software/util/type_map/type_map.tpp"
