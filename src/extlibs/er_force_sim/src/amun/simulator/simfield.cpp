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

// the framework treats a field with both boundary widths set to zero as one that is
// played without a boundary area
static bool hasNoBoundary(const world::Geometry& geometry)
{
    return geometry.boundary_width() == 0.0 && geometry.boundary_width_goal_line() == 0.0;
}

SimField::SimField(std::shared_ptr<btDiscreteDynamicsWorld> world,
                   const world::Geometry& geometry)
    : m_world(world)
{
    const bool noBoundary = hasNoBoundary(geometry);

    const float totalWidth = geometry.field_width() / 2.0f + geometry.boundary_width();
    // the boundary behind the goal lines can differ from the one along the touch lines
    const float boundaryWidthGoalLine = geometry.has_boundary_width_goal_line()
                                            ? geometry.boundary_width_goal_line()
                                            : geometry.boundary_width();
    const float totalHeight   = geometry.field_height() / 2.0f + boundaryWidthGoalLine;
    const float roomHeight    = 8.0f;  // Upper boundary of "room" that the field lives in
    const float height        = geometry.field_height() / 2.0f - geometry.line_width();
    const float goalWidthHalf = geometry.goal_width() / 2.0f + geometry.goal_wall_width();
    const float goalHeightHalf = geometry.goal_height() / 2.0f;
    const float goalDepth      = geometry.goal_depth() + geometry.goal_wall_width();
    // goal side walls need to extend to the wall
    const float goalSideWallLengthHalf = boundaryWidthGoalLine / 2.0f;
    const float goalWallHalf           = geometry.goal_wall_width() / 2.0f;

    // Collision shapes
    m_plane    = std::make_unique<btStaticPlaneShape>(btVector3(0, 0, 1), 0);
    m_goalSide = std::make_unique<btBoxShape>(
        btVector3(goalWallHalf, goalSideWallLengthHalf, goalHeightHalf) * SIMULATOR_SCALE);
    m_goalBack = std::make_unique<btBoxShape>(
        btVector3(goalWidthHalf, goalWallHalf, goalHeightHalf) * SIMULATOR_SCALE);

    // Create the "room" that the field lives in
    // Floor
    addObject(m_plane.get(),
              btTransform(btQuaternion(btVector3(1, 0, 0), 0),
                          btVector3(0, 0, 0) * SIMULATOR_SCALE),
              0.56, 0.35);
    // Roof
    addObject(m_plane.get(),
              btTransform(btQuaternion(btVector3(1, 0, 0), M_PI),
                          btVector3(0, 0, roomHeight) * SIMULATOR_SCALE),
              0.3, 0.35);

    // Walls behind the goal lines
    if (noBoundary)
    {
        // the walls on the goal lines must not block the goals, so they are
        // built from boxes on either side of the goal instead of a single plane
        const float goalLineBoundaryWidthHalf = 0.5f * (totalWidth - goalWidthHalf);
        m_goalLineBoundaryShape               = std::make_unique<btBoxShape>(
            btVector3(goalLineBoundaryWidthHalf, 0.5f, roomHeight * 0.5f) *
            SIMULATOR_SCALE);
        const float shapeOffsetX = goalWidthHalf + goalLineBoundaryWidthHalf;
        const btQuaternion identity = btQuaternion::getIdentity();
        for (const float side : {-1.0f, 1.0f})
        {
            // the boxes extend away from the field
            const btVector3 intoVoid = btVector3(0, side * 0.5f, roomHeight * 0.5f) *
                                       SIMULATOR_SCALE;
            for (const float sideX : {-1.0f, 1.0f})
            {
                addObject(m_goalLineBoundaryShape.get(),
                          btTransform(identity,
                                      intoVoid + btVector3(sideX * shapeOffsetX,
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

    // Corner blocks to smooth out the edges. On the actual field they are triangular
    // blocks put in the corners of the field, but here they are approximated by the
    // one side that is relevant for collision checking
    if (geometry.has_corner_block_cathetus_length())
    {
        const float cathetus     = geometry.corner_block_cathetus_length();
        const float hypothenuse  = std::sqrt(2 * cathetus * cathetus);
        // basically the height of the triangle when looking top down onto the field
        const float blockOffset = (cathetus * cathetus) / hypothenuse;
        m_cornerBlockShape      = std::make_unique<btBoxShape>(
            btVector3(hypothenuse * 0.5f, goalWallHalf, roomHeight * 0.5f) *
            SIMULATOR_SCALE);

        // The blocks are placed in order of angle. Half of them can be mirrored with a
        // 180 degree rotation, so only M_PI / 4 and 3 * M_PI / 4 are listed and, if
        // negativeYHalf is set, the same positions are additionally rotated
        for (const bool negativeYHalf : {false, true})
        {
            for (const auto& [multipleOfPi, mirrorX] :
                 {std::pair{0.25, false}, std::pair{0.75, true}})
            {
                // for the same rotation the triangle is e.g. at the lower left corner
                // of the field and at the upper left corner of the goal, so the signs
                // of totalWidthOffset and goalWidthOffset are flipped
                const float totalWidthOffset = mirrorX ? totalWidth : -totalWidth;
                const float goalWidthOffset  = mirrorX ? -goalWidthHalf : goalWidthHalf;

                const btVector3 shapeOffsetFromCorner =
                    btVector3(blockOffset, 0, roomHeight * 0.5f)
                        .rotate(btVector3(0, 0, 1), M_PI * -multipleOfPi) *
                    SIMULATOR_SCALE;
                const btTransform baseTransform(
                    btQuaternion(btVector3(0, 0, 1), M_PI * multipleOfPi),
                    shapeOffsetFromCorner);
                const btTransform flip(btQuaternion(btVector3(0, 0, 1), M_PI));

                btTransform cornerTransform = baseTransform;
                cornerTransform.getOrigin() +=
                    btVector3(totalWidthOffset, totalHeight, 0) * SIMULATOR_SCALE;
                if (negativeYHalf)
                {
                    cornerTransform = flip * cornerTransform;
                }
                addObject(m_cornerBlockShape.get(), cornerTransform, 0.3, 0.35);

                // blocks around the goals are only needed when the field has a boundary
                if (!noBoundary)
                {
                    btTransform goalTransform = baseTransform;
                    goalTransform.getOrigin() +=
                        btVector3(goalWidthOffset, totalHeight, 0) * SIMULATOR_SCALE;
                    if (negativeYHalf)
                    {
                        goalTransform = flip * goalTransform;
                    }
                    addObject(m_cornerBlockShape.get(), goalTransform, 0.3, 0.35);
                }
            }
        }
    }

    // Create goals
    for (const float side : {-1.0f, 1.0f})
    {
        // with a boundary the goal stands on the outside of the line, so it is
        // offset by half the line width
        const float lineWidthOffset = noBoundary ? 0.0f : geometry.line_width() * 0.5f;
        const float sideWallY = side * (height + goalSideWallLengthHalf + lineWidthOffset);

        addObject(m_goalSide.get(),
                  btTransform(btQuaternion::getIdentity(),
                              btVector3(goalWidthHalf - goalWallHalf, sideWallY,
                                        goalHeightHalf) *
                                  SIMULATOR_SCALE),
                  0.3, 0.5);
        addObject(m_goalSide.get(),
                  btTransform(btQuaternion::getIdentity(),
                              btVector3(-(goalWidthHalf - goalWallHalf), sideWallY,
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
