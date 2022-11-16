#pragma once

#include <vector>

template <class T, typename Func>
std::vector<T> nearestNeighbours(const T& candidate, const std::vector<T>& neighbors,
                                 double radius, Func comparator)
{
    std::vector<T> robot_subset;
    for (const T& neighbor : neighbors)
    {
        if (comparator(candidate, neighbor) < radius * radius &&
            candidate != neighbor)
        {
            robot_subset.push_back(neighbor);
        }
    }
    return robot_subset;
}
