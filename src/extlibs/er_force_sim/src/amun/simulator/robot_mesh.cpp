#include "robot_mesh.h"

#include <cmath>

std::vector<std::tuple<float, float, float>> generateRobotShellPoints(
    unsigned int num_segments, float start_angle, float end_angle, float radius,
    float height)
{
    std::vector<std::tuple<float, float, float>> points;

    float angle            = start_angle;
    const float angle_step = (end_angle - start_angle) / num_segments;

    for (unsigned int i = 0; i <= num_segments; ++i)
    {
        const float x = radius * std::cos(angle);
        const float y = radius * std::sin(angle);

        points.emplace_back(x, y, height / 2.0f);
        points.emplace_back(x, y, -height / 2.0f);

        angle += angle_step;
    }

    return points;
}

std::vector<std::vector<std::tuple<float, float, float>>> createRobotMesh(
    float radius, float height, float angle, float hole_depth, float hole_height)
{
    constexpr unsigned int NUM_SEGMENTS_HULL   = 20;
    constexpr unsigned int NUM_SEGMENTS_PILLAR = 5;

    const float front_plate_length = std::sin(angle / 2.0) * radius;
    const float front_plate_pos    = radius * std::cos(angle / 2.0);
    const float hole_plate_pos     = front_plate_pos - hole_depth;
    const float outer_angle        = std::acos(hole_plate_pos / radius) * 2;
    const float angle_diff         = (outer_angle - angle) / 2.0;
    const float half_outer_angle   = outer_angle / 2.0;
    const float outer_angle_start  = half_outer_angle + M_PI_2;
    const float outer_angle_stop   = 2.0 * M_PI - half_outer_angle + M_PI_2;

    std::vector<std::vector<std::tuple<float, float, float>>> mesh_parts;

    // Main hull
    mesh_parts.push_back(generateRobotShellPoints(NUM_SEGMENTS_HULL, outer_angle_start,
                                                  outer_angle_stop, radius, height));

    // Left pillar
    auto left_pillar =
        generateRobotShellPoints(NUM_SEGMENTS_PILLAR, outer_angle_stop,
                                 outer_angle_stop + angle_diff, radius, height);
    left_pillar.emplace_back(front_plate_length, hole_plate_pos, height / 2.0);
    left_pillar.emplace_back(front_plate_length, hole_plate_pos, -height / 2.0);
    mesh_parts.push_back(left_pillar);

    // Right pillar
    auto right_pillar =
        generateRobotShellPoints(NUM_SEGMENTS_PILLAR, outer_angle_start - angle_diff,
                                 outer_angle_start, radius, height);
    right_pillar.emplace_back(-front_plate_length, hole_plate_pos, height / 2.0);
    right_pillar.emplace_back(-front_plate_length, hole_plate_pos, -height / 2.0);
    mesh_parts.push_back(right_pillar);

    // The remaining front plate and dribbler "hole"
    mesh_parts.push_back({
        {front_plate_length, hole_plate_pos, height / 2.0},
        {-front_plate_length, hole_plate_pos, height / 2.0},
        {front_plate_length, hole_plate_pos, -height / 2.0 + hole_height},
        {-front_plate_length, hole_plate_pos, -height / 2.0 + hole_height},
        {front_plate_length, front_plate_pos, height / 2.0},
        {-front_plate_length, front_plate_pos, height / 2.0},
        {front_plate_length, front_plate_pos, -height / 2.0 + hole_height},
        {-front_plate_length, front_plate_pos, -height / 2.0 + hole_height},
    });

    return mesh_parts;
}
