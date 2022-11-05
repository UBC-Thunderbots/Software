#include "brute_force_nearest_neighbor_search.h"

template <class T, typename F>
std::vector<T> nearestNeighbours(const T& this_robot, const std::vector<T>& input,
                                 double radius, F comparator)
{
    std::vector<T> robot_subset;
    for (const T& candidate_robot : input)
    {
        if (comparator(this_robot, candidate_robot) < radius * radius &&
            this_robot != candidate_robot)
        {
            robot_subset.push_back(candidate_robot);
        }
    }
    return robot_subset;
}
