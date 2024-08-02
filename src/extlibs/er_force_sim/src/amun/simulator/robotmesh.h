#ifndef ROBOTMESH_H
#define ROBOTMESH_H

#include <tuple>
#include <vector>

namespace camun
{
    namespace simulator
    {
        std::vector<std::vector<std::tuple<float, float, float>>> createRobotMesh(
            float radius, float height, float angle, float holeDepth, float holeHeight);
    }  // namespace simulator
}  // namespace camun

#endif  // ROBOTMESH_H
