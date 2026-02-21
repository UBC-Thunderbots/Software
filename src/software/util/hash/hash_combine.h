#pragma once

#include <functional>
#include <cstddef>


/**
* Combine hash values of multiple objects
* 
* @param seed current hash value to be updated
* @param v object to add to the current hash value
* NOTE: needs to ensure type T has std::hash template function implemented
*
* https://stackoverflow.com/questions/7222143/unordered-map-hash-function-c
*/
template <class T>
inline void hashCombine(std::size_t& seed, const T& v) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

