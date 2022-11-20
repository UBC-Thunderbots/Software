#pragma once

#include <vector>

/**
 * Finds the neighbors of a candidate based on a comparator and radius
 *
 * @tparam T generic type for the candidate and its neighbors
 * @tparam Func type for the comparator
 * @param candidate cannot be null
 * @param neighbors a list of T objects neighbors, which may contain the candidate object
 * itself
 * @param radius the maximum distance from a candidate object
 * @param comparator function that measures the square distance between a candidate and a
 * neighbor
 * @return a list representing a subset of neighbors
 */
template <class T, typename Func>
std::vector<T> nearestNeighbours(const T& candidate, const std::vector<T>& neighbors,
                                 double radius, Func comparator)
{
    std::vector<T> subset;
    for (const T& neighbor : neighbors)
    {
        if (comparator(candidate, neighbor) < radius * radius && candidate != neighbor)
        {
            subset.push_back(neighbor);
        }
    }
    return subset;
}
