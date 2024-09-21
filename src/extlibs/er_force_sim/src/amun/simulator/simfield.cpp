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

#include "simulator.h"

using namespace camun::simulator;

SimField::SimField(std::shared_ptr<btDiscreteDynamicsWorld> world,
                   const world::Geometry &geometry)
    : m_world(world)
{
    const float totalWidth  = geometry.field_width() / 2.0f + geometry.boundary_width();
    const float totalHeight = geometry.field_height() / 2.0f + geometry.boundary_width();
    const float roomHeight  = 8.0f;  // Upper boundary of "room" that the field lives in
    const float height      = geometry.field_height() / 2.0f - geometry.line_width();
    const float goalWidthHalf = geometry.goal_width() / 2.0f + geometry.goal_wall_width();
    const float goalHeightHalf = geometry.goal_height() / 2.0f;
    const float goalDepth      = geometry.goal_depth() + geometry.goal_wall_width();
    const float goalDepthHalf  = goalDepth / 2.0f;
    const float goalWallHalf   = geometry.goal_wall_width() / 2.0f;

    // Collision shapes
    m_plane    = std::make_unique<btStaticPlaneShape>(btVector3(0, 0, 1), 0);
    m_goalSide = std::make_unique<btBoxShape>(
        btVector3(goalWallHalf, goalDepthHalf, goalHeightHalf) * SIMULATOR_SCALE);
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
    // Walls
    addObject(m_plane.get(),
              btTransform(btQuaternion(btVector3(1, 0, 0), M_PI_2),
                          btVector3(0, totalHeight, 0) * SIMULATOR_SCALE),
              0.3, 0.35);
    addObject(m_plane.get(),
              btTransform(btQuaternion(btVector3(1, 0, 0), -M_PI_2),
                          btVector3(0, -totalHeight, 0) * SIMULATOR_SCALE),
              0.3, 0.35);
    addObject(m_plane.get(),
              btTransform(btQuaternion(btVector3(0, 1, 0), M_PI_2),
                          btVector3(-totalWidth, 0, 0) * SIMULATOR_SCALE),
              0.3, 0.35);
    addObject(m_plane.get(),
              btTransform(btQuaternion(btVector3(0, 1, 0), -M_PI_2),
                          btVector3(totalWidth, 0, 0) * SIMULATOR_SCALE),
              0.3, 0.35);

    // Create goals
    for (const float side : {-1.0f, 1.0f})
    {
        addObject(m_goalSide.get(),
                  btTransform(btQuaternion::getIdentity(),
                              btVector3((goalWidthHalf - goalWallHalf),
                                        side * (height + goalDepthHalf), goalHeightHalf) *
                                  SIMULATOR_SCALE),
                  0.3, 0.5);
        addObject(m_goalSide.get(),
                  btTransform(btQuaternion::getIdentity(),
                              btVector3(-(goalWidthHalf - goalWallHalf),
                                        side * (height + goalDepthHalf), goalHeightHalf) *
                                  SIMULATOR_SCALE),
                  0.3, 0.5);
        addObject(m_goalBack.get(),
                  btTransform(btQuaternion::getIdentity(),
                              btVector3(0.0f, side * (height + goalDepth - goalWallHalf),
                                        goalHeightHalf) *
                                  SIMULATOR_SCALE),
                  0.1, 0.5);
    }
}

SimField::~SimField()
{
    for (const auto &object : m_objects)
    {
        m_world->removeCollisionObject(object.get());
    }
}

void SimField::addObject(btCollisionShape *shape, const btTransform &transform,
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
