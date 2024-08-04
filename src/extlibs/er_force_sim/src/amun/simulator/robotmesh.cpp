/***************************************************************************
 *   Copyright 2015 Michael Eischer, Philipp Nordhus                       *
 *   Robotics Erlangen e.V.                                                *
 *   http://www.robotics-erlangen.de/                                      *
 *   info@robotics-erlangen.de                                             *
 *                                                                         *
 *   This program is free software: you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation, either version 3 of the License, or     *
 *   any later version.                                                    *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

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

    // Diagram of robot (top view)
    //    
    //                 ┌────────┬────frontPlatePos                                                                 
    //                 ┌───┬─────────holePlatePos                                                                 
    //                     ┌────┬────holeDepth                                                                 
    //             , - ~ - ,                        
    //         , '        .  ' ,                     
    //       ,           .     .| ─┐                 
    //      ,           .   .   |  │                 
    //     ,           .\.      |  │                 
    //     ,           + )─┐    |  ├─frontPlateLength
    //     ,           `/` │    |  │                 
    //      ,           │  │'   |  │                 
    //       ,          │` │   .|  │                 
    //         ,        │ `│  , ' ─┘                 
    //           ' - , _│, │'                        
    //                  │  └─angle                                             
    //                  └────angleDiff
    //                       outerAngle = angle + angleDiff + angleDiff   
    //    
    const float frontPlateLength = std::sin(angle / 2.0) * radius;
    const float frontPlatePos    = radius * std::cos(angle / 2.0);
    const float holePlatePos     = frontPlatePos - holeDepth;
    const float outerAngle       = std::acos(holePlatePos / radius) * 2;
    const float angleDiff        = (outerAngle - angle) / 2.0;
    const float halfOuterAngle   = outerAngle / 2.0;
    const float outerAngleStart  = halfOuterAngle + M_PI_2;
    const float outerAngleStop   = 2.0 * M_PI - halfOuterAngle + M_PI_2;

    std::vector<std::vector<std::tuple<float, float, float>>> meshParts;

    // Parts of robot (top view)
    //    
    //             , - ~ - ,    Right pillar                    
    //         , '         | ' ,                       
    //       ,             ├────┐                 
    //      ,              |    |                 
    //     ,               |    |                 
    //     , Main hull     |  Front plate and dribbler hole
    //     ,               |    |                 
    //      ,              |    |                 
    //       ,             ├────┘                 
    //         ,           |  .'                 
    //           ' - , _  , '   Left pillar                     
    //    

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
