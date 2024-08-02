#include "robotmesh.h"

#include <cmath>

using namespace camun::simulator;

static std::vector<std::tuple<float, float, float>> generateRobotShellPoints(
    unsigned int numSegments, float startAngle, float endAngle, float radius,
    float height)
{
    std::vector<std::tuple<float, float, float>> points;

    float angle           = startAngle;
    const float angleStep = (endAngle - startAngle) / numSegments;

    for (unsigned int i = 0; i <= numSegments; ++i)
    {
        const float x = radius * std::cos(angle);
        const float y = radius * std::sin(angle);

        points.emplace_back(x, y, height / 2.0);
        points.emplace_back(x, y, -height / 2.0);

        angle += angleStep;
    }

    return points;
}

std::vector<std::vector<std::tuple<float, float, float>>>
camun::simulator::createRobotMesh(float radius, float height, float angle,
                                  float holeDepth, float holeHeight)
{
    static constexpr unsigned int NUM_SEGMENTS_HULL   = 20;
    static constexpr unsigned int NUM_SEGMENTS_PILLAR = 5;

    const float frontPlateLength = std::sin(angle / 2.0) * radius;
    const float frontPlatePos    = radius * std::cos(angle / 2.0);
    const float holePlatePos     = frontPlatePos - holeDepth;
    const float outerAngle       = std::acos(holePlatePos / radius) * 2;
    const float angleDiff        = (outerAngle - angle) / 2.0;
    const float halfOuterAngle   = outerAngle / 2.0;
    const float outerAngleStart  = halfOuterAngle + M_PI_2;
    const float outerAngleStop   = 2.0 * M_PI - halfOuterAngle + M_PI_2;

    std::vector<std::vector<std::tuple<float, float, float>>> meshParts;

    // Main hull
    meshParts.push_back(generateRobotShellPoints(NUM_SEGMENTS_HULL, outerAngleStart,
                                                 outerAngleStop, radius, height));

    // Left pillar
    auto leftPillar = generateRobotShellPoints(
        NUM_SEGMENTS_PILLAR, outerAngleStop, outerAngleStop + angleDiff, radius, height);
    leftPillar.emplace_back(frontPlateLength, holePlatePos, height / 2.0);
    leftPillar.emplace_back(frontPlateLength, holePlatePos, -height / 2.0);
    meshParts.push_back(leftPillar);

    // Right pillar
    auto rightPillar =
        generateRobotShellPoints(NUM_SEGMENTS_PILLAR, outerAngleStart - angleDiff,
                                 outerAngleStart, radius, height);
    rightPillar.emplace_back(-frontPlateLength, holePlatePos, height / 2.0);
    rightPillar.emplace_back(-frontPlateLength, holePlatePos, -height / 2.0);
    meshParts.push_back(rightPillar);

    // The remaining front plate and dribbler "hole"
    meshParts.push_back({
        {frontPlateLength, holePlatePos, height / 2.0},
        {-frontPlateLength, holePlatePos, height / 2.0},
        {frontPlateLength, holePlatePos, -height / 2.0 + holeHeight},
        {-frontPlateLength, holePlatePos, -height / 2.0 + holeHeight},
        {frontPlateLength, frontPlatePos, height / 2.0},
        {-frontPlateLength, frontPlatePos, height / 2.0},
        {frontPlateLength, frontPlatePos, -height / 2.0 + holeHeight},
        {-frontPlateLength, frontPlatePos, -height / 2.0 + holeHeight},
    });

    return meshParts;
}
