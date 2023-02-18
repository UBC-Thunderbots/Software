#pragma once

#include <vector>

/**
 * Finds the neighbors of a candidate based on a comparator and radius.
 * Neighbors on the edge of the radius are included.
 *
 * @tparam T generic type for the candidate and its neighbors
 * @tparam Func type for the function comparator. Must be in the form `double f(const T&,
 * const T&)`
 * @param candidate cannot be null
 * @param neighbors a list of T objects neighbors, which may contain the candidate object
 * itself
 * @param threshold the maximum distance from a candidate object
 * @param distanceFunc function that measures the distance between a candidate and a
 * neighbor
 * @return a list representing a subset of neighbors
 */
template <class T, typename Func>
// std::iterator
std::vector<T> findNeighboursInThreshold(const T& candidate,
                                         const std::vector<T>& neighbors,
                                         double threshold, Func distanceFunc)
{
    std::vector<T> subset;
    for (const T& neighbor : neighbors)
    {
        if (distanceFunc(candidate, neighbor) <= threshold)
        {
            subset.push_back(neighbor);
        }
    }
    return subset;
}
