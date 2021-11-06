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

SimField::SimField(btDiscreteDynamicsWorld *world, const world::Geometry &geometry)
    : m_world(world)
{
    const float totalWidth  = geometry.field_width() / 2.0f + geometry.boundary_width();
    const float totalHeight = geometry.field_height() / 2.0f + geometry.boundary_width();
    // upper boundary
    const float roomHeight    = 8.0f;
    const float height        = geometry.field_height() / 2.0f - geometry.line_width();
    const float goalWidthHalf = geometry.goal_width() / 2.0f + geometry.goal_wall_width();
    const float goalHeightHalf = geometry.goal_height() / 2.0f;
    const float goalDepth      = geometry.goal_depth() + geometry.goal_wall_width();
    const float goalDepthHalf  = goalDepth / 2.0f;
    const float goalWallHalf   = geometry.goal_wall_width() / 2.0f;

    // obstacle prototypes
    m_plane    = new btStaticPlaneShape(btVector3(0, 0, 1), 0);
    m_goalSide = new btBoxShape(btVector3(goalWallHalf, goalDepthHalf, goalHeightHalf) *
                                SIMULATOR_SCALE);
    m_goalBack = new btBoxShape(btVector3(goalWidthHalf, goalWallHalf, goalHeightHalf) *
                                SIMULATOR_SCALE);

    // build field cube
    // floor
    addObject(m_plane,
              btTransform(btQuaternion(btVector3(1, 0, 0), 0),
                          btVector3(0, 0, 0) * SIMULATOR_SCALE),
              0.56, 0.35);
    // others
    addObject(m_plane,
              btTransform(btQuaternion(btVector3(1, 0, 0), M_PI),
                          btVector3(0, 0, roomHeight) * SIMULATOR_SCALE),
              0.3, 0.35);

    addObject(m_plane,
              btTransform(btQuaternion(btVector3(1, 0, 0), M_PI_2),
                          btVector3(0, totalHeight, 0) * SIMULATOR_SCALE),
              0.3, 0.35);
    addObject(m_plane,
              btTransform(btQuaternion(btVector3(1, 0, 0), -M_PI_2),
                          btVector3(0, -totalHeight, 0) * SIMULATOR_SCALE),
              0.3, 0.35);

    addObject(m_plane,
              btTransform(btQuaternion(btVector3(0, 1, 0), M_PI_2),
                          btVector3(-totalWidth, 0, 0) * SIMULATOR_SCALE),
              0.3, 0.35);
    addObject(m_plane,
              btTransform(btQuaternion(btVector3(0, 1, 0), -M_PI_2),
                          btVector3(totalWidth, 0, 0) * SIMULATOR_SCALE),
              0.3, 0.35);

    // create goals
    for (int goal = 0; goal < 2; goal++)
    {
        const float side       = (goal == 0) ? -1.0f : 1.0f;
        const btQuaternion rot = btQuaternion::getIdentity();

        addObject(
            m_goalSide,
            btTransform(rot, btVector3((goalWidthHalf - goalWallHalf),
                                       side * (height + goalDepthHalf), goalHeightHalf) *
                                 SIMULATOR_SCALE),
            0.3, 0.5);
        addObject(
            m_goalSide,
            btTransform(rot, btVector3(-(goalWidthHalf - goalWallHalf),
                                       side * (height + goalDepthHalf), goalHeightHalf) *
                                 SIMULATOR_SCALE),
            0.3, 0.5);
        addObject(
            m_goalBack,
            btTransform(rot, btVector3(0.0f, side * (height + goalDepth - goalWallHalf),
                                       goalHeightHalf) *
                                 SIMULATOR_SCALE),
            0.1, 0.5);
    }
}

SimField::~SimField()
{
    foreach (btCollisionObject *object, m_objects)
    {
        m_world->removeCollisionObject(object);
        delete object;
    }

    delete m_goalSide;
    delete m_goalBack;
    delete m_plane;
}

void SimField::addObject(btCollisionShape *shape, const btTransform &transform,
                         float restitution, float friction)
{
    // create new obstacle
    btCollisionObject *object = new btCollisionObject;
    object->setCollisionShape(shape);
    // damp ball a bit if it hits an obstacle
    object->setRestitution(restitution);
    // the friction is multiplied with the colliding obstacle ones
    object->setFriction(friction);
    object->setRollingFriction(friction);
    object->setWorldTransform(transform);
    m_world->addCollisionObject(object);
    m_objects.append(object);
}
