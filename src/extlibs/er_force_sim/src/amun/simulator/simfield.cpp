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

#include "simfield.h"

#include <cmath>
#include <utility>

#include "simulator.h"

using namespace camun::simulator;

/**
 * Whether the field is played on without any boundary area around it, in which case the
 * walls stand right at the field lines and the goals stand outside of them
 *
 * @param geometry the field geometry
 *
 * @return true if the field has no boundary area
 */
static bool hasNoBoundaryArea(const world::Geometry& geometry)
{
    return geometry.boundary_width() == 0.0f &&
           geometry.boundary_width_goal_line() == 0.0f;
}

SimField::SimField(std::shared_ptr<btDiscreteDynamicsWorld> world,
                   const world::Geometry& geometry)
    : m_world(world)
{
    // The boundary behind the goal lines may be wider than the one along the touch
    // lines. Fields that do not report it use the same width all around.
    const float boundaryWidthGoalLine = geometry.has_boundary_width_goal_line()
                                            ? geometry.boundary_width_goal_line()
                                            : geometry.boundary_width();

    const float totalWidth    = geometry.field_width() / 2.0f + geometry.boundary_width();
    const float totalHeight   = geometry.field_height() / 2.0f + boundaryWidthGoalLine;
    const float roomHeight    = 8.0f;  // Upper boundary of "room" that the field lives in
    const float height        = geometry.field_height() / 2.0f - geometry.line_width();
    const float goalWidthHalf = geometry.goal_width() / 2.0f + geometry.goal_wall_width();
    const float goalHeightHalf = geometry.goal_height() / 2.0f;
    const float goalDepth      = geometry.goal_depth() + geometry.goal_wall_width();
    const float goalWallHalf   = geometry.goal_wall_width() / 2.0f;
    // The side walls of the goal extend all the way back to the field wall
    const float goalSideWallLengthHalf = boundaryWidthGoalLine / 2.0f;

    // Collision shapes
    m_plane    = std::make_unique<btStaticPlaneShape>(btVector3(0, 0, 1), 0);
    m_goalSide = std::make_unique<btBoxShape>(
        btVector3(goalWallHalf, goalSideWallLengthHalf, goalHeightHalf) *
        SIMULATOR_SCALE);
    m_goalBack = std::make_unique<btBoxShape>(
        btVector3(goalWidthHalf, goalWallHalf, goalHeightHalf) * SIMULATOR_SCALE);

    // Create the "room" that the field lives in
    // Floor
    addObject(m_plane.get(),
              btTransform(btQuaternion(btVector3(1, 0, 0), 0),
                          btVector3(0, 0, 0) * SIMULATOR_SCALE),
              FLOOR_RESTITUTION, FLOOR_FRICTION);
    // Roof
    addObject(m_plane.get(),
              btTransform(btQuaternion(btVector3(1, 0, 0), M_PI),
                          btVector3(0, 0, roomHeight) * SIMULATOR_SCALE),
              0.3, 0.35);
    // Walls behind the goal lines. Without a boundary area the goals themselves stand
    // where the wall would be, so the wall is split into one block on each side of them.
    if (hasNoBoundaryArea(geometry))
    {
        const float goalLineBoundaryWidthHalf = 0.5f * (totalWidth - goalWidthHalf);
        m_goalLineBoundary                    = std::make_unique<btBoxShape>(
            btVector3(goalLineBoundaryWidthHalf, 0.5f, roomHeight * 0.5f) *
            SIMULATOR_SCALE);
        const float shapeOffsetX = goalWidthHalf + goalLineBoundaryWidthHalf;

        for (const float side : {-1.0f, 1.0f})
        {
            // offset the blocks so that they start at the goal line and extend away
            // from the field
            const btVector3 shapeOffsetIntoVoid =
                btVector3(0, side * 0.5f, roomHeight * 0.5f) * SIMULATOR_SCALE;
            for (const float xSide : {-1.0f, 1.0f})
            {
                addObject(
                    m_goalLineBoundary.get(),
                    btTransform(btQuaternion::getIdentity(),
                                shapeOffsetIntoVoid + btVector3(xSide * shapeOffsetX,
                                                                side * totalHeight, 0) *
                                                          SIMULATOR_SCALE),
                    0.3, 0.35);
            }
        }
    }
    else
    {
        addObject(m_plane.get(),
                  btTransform(btQuaternion(btVector3(1, 0, 0), M_PI_2),
                              btVector3(0, totalHeight, 0) * SIMULATOR_SCALE),
                  0.3, 0.35);
        addObject(m_plane.get(),
                  btTransform(btQuaternion(btVector3(1, 0, 0), -M_PI_2),
                              btVector3(0, -totalHeight, 0) * SIMULATOR_SCALE),
                  0.3, 0.35);
    }

    // Walls along the touch lines
    addObject(m_plane.get(),
              btTransform(btQuaternion(btVector3(0, 1, 0), M_PI_2),
                          btVector3(-totalWidth, 0, 0) * SIMULATOR_SCALE),
              0.3, 0.35);
    addObject(m_plane.get(),
              btTransform(btQuaternion(btVector3(0, 1, 0), -M_PI_2),
                          btVector3(totalWidth, 0, 0) * SIMULATOR_SCALE),
              0.3, 0.35);

    // Blocks that smooth out the corners of the field. On the real field these are
    // triangular blocks put into the corners; here only the one side of them that
    // matters for collision checking is simulated.
    if (geometry.has_corner_block_cathetus_length())
    {
        const float cathetus   = geometry.corner_block_cathetus_length();
        const float hypotenuse = std::sqrt(2 * cathetus * cathetus);
        // the height of the triangle when looking top down onto the field
        const float blockOffset = (cathetus * cathetus) / hypotenuse;
        m_cornerBlock           = std::make_unique<btBoxShape>(
            btVector3(hypotenuse * 0.5f, goalWallHalf, roomHeight * 0.5f) *
            SIMULATOR_SCALE);

        // The blocks are placed in order of angle. Half of them are just the other half
        // rotated by 180 degrees, so only the angles of the positive y half are listed
        // and negativeYHalf rotates those by 180 degrees.
        for (const bool negativeYHalf : {false, true})
        {
            for (const auto& [multipleOfPi, mirrorX] :
                 {std::pair{0.25f, false}, std::pair{0.75f, true}})
            {
                // the signs of the two offsets are flipped relative to each other,
                // because for the same rotation the triangle is for example at the
                // lower left corner of the field but at the upper left corner of the
                // goal
                const float totalWidthOffset = mirrorX ? totalWidth : -totalWidth;
                const float goalWidthOffset  = mirrorX ? -goalWidthHalf : goalWidthHalf;

                const btVector3 shapeOffsetFromCorner =
                    btVector3(blockOffset, 0, roomHeight * 0.5f)
                        .rotate(btVector3(0, 0, 1), M_PI * -multipleOfPi) *
                    SIMULATOR_SCALE;
                const btTransform baseTransform(
                    btQuaternion(btVector3(0, 0, 1), M_PI * multipleOfPi),
                    shapeOffsetFromCorner);

                const auto mirrorIntoYHalf = [negativeYHalf](btTransform transform)
                {
                    if (negativeYHalf)
                    {
                        return btTransform(btQuaternion(btVector3(0, 0, 1), M_PI)) *
                               transform;
                    }
                    return transform;
                };

                btTransform cornerTransform = baseTransform;
                cornerTransform.getOrigin() +=
                    btVector3(totalWidthOffset, totalHeight, 0) * SIMULATOR_SCALE;
                addObject(m_cornerBlock.get(), mirrorIntoYHalf(cornerTransform), 0.3,
                          0.35);

                // a field with a boundary area also has blocks around the goals
                if (!hasNoBoundaryArea(geometry))
                {
                    btTransform goalTransform = baseTransform;
                    goalTransform.getOrigin() +=
                        btVector3(goalWidthOffset, totalHeight, 0) * SIMULATOR_SCALE;
                    addObject(m_cornerBlock.get(), mirrorIntoYHalf(goalTransform), 0.3,
                              0.35);
                }
            }
        }
    }

    // Create goals. Without a boundary area the goal stands on the outside of the goal
    // line rather than on its middle, so it is offset by half a line width.
    const float lineWidthOffset =
        hasNoBoundaryArea(geometry) ? 0.0f : geometry.line_width() * 0.5f;

    for (const float side : {-1.0f, 1.0f})
    {
        addObject(m_goalSide.get(),
                  btTransform(btQuaternion::getIdentity(),
                              btVector3((goalWidthHalf - goalWallHalf),
                                        side * (height + goalSideWallLengthHalf +
                                                lineWidthOffset),
                                        goalHeightHalf) *
                                  SIMULATOR_SCALE),
                  0.3, 0.5);
        addObject(m_goalSide.get(),
                  btTransform(btQuaternion::getIdentity(),
                              btVector3(-(goalWidthHalf - goalWallHalf),
                                        side * (height + goalSideWallLengthHalf +
                                                lineWidthOffset),
                                        goalHeightHalf) *
                                  SIMULATOR_SCALE),
                  0.3, 0.5);
        addObject(m_goalBack.get(),
                  btTransform(btQuaternion::getIdentity(),
                              btVector3(0.0f,
                                        side * (height + goalDepth - goalWallHalf +
                                                lineWidthOffset),
                                        goalHeightHalf) *
                                  SIMULATOR_SCALE),
                  0.1, 0.5);
    }
}

SimField::~SimField()
{
    for (const auto& object : m_objects)
    {
        m_world->removeCollisionObject(object.get());
    }
}

void SimField::addObject(btCollisionShape* shape, const btTransform& transform,
                         float restitution, float friction)
{
    std::unique_ptr<btCollisionObject> object = std::make_unique<btCollisionObject>();

    object->setCollisionShape(shape);
    object->setWorldTransform(transform);
    object->setRestitution(restitution);
    object->setFriction(friction);
    object->setRollingFriction(friction);

    m_world->addCollisionObject(object.get());
    m_objects.push_back(std::move(object));
}
