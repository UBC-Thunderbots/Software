#pragma once

#include <vector>

template<class T, class F>
static std::vector<T> nearestNeighbours(const T& this_robot, const std::vector<T>& input, double radius, F comparator);
