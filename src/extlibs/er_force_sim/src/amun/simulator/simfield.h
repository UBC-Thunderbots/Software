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

#ifndef SIMFIELD_H
#define SIMFIELD_H

#include <btBulletDynamicsCommon.h>

#include "extlibs/er_force_sim/src/protobuf/world.pb.h"

namespace camun
{
namespace simulator
{
class SimField;
}  // namespace simulator
}  // namespace camun

class camun::simulator::SimField
{
   public:
    SimField(std::shared_ptr<btDiscreteDynamicsWorld> world,
             const world::Geometry &geometry);
    ~SimField();

   private:
    void addObject(btCollisionShape *shape, const btTransform &transform,
                   float restitution, float friction);

   private:
    std::shared_ptr<btDiscreteDynamicsWorld> m_world;
    std::unique_ptr<btCollisionShape> m_plane;
    std::unique_ptr<btCollisionShape> m_goalSide;
    std::unique_ptr<btCollisionShape> m_goalBack;
    std::vector<std::unique_ptr<btCollisionObject>> m_objects;
};

#endif  // SIMFIELD_H
