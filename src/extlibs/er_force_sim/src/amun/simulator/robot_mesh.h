#pragma once

#include <vector>
#include <tuple>

std::vector<std::vector<std::tuple<float, float, float>>> createRobotMesh(
    float radius, float height, float angle, float hole_depth, float hole_height);
