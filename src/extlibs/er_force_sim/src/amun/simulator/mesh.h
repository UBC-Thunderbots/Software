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

#ifndef MESH_H
#define MESH_H

#include <QtCore/QMap>
#include <QtCore/QString>
#include <QtCore/QVector>
#include <QtGui/QVector3D>

namespace camun
{
    namespace simulator
    {
        class Mesh;
    }
}  // namespace camun

class camun::simulator::Mesh
{
   public:
    Mesh(float radius, float height, float angle, float holeSize, float boxHeight);
    void createRobotMeshes(float radius, float height, float angle);
    const QList<QList<QVector3D>> &hull() const
    {
        return m_hull;
    }

   private:
    void addRobotCover(uint num, float startAngle, float endAngle);
    void addSidePart(uint num, float angleStart, float angleStop, bool right);

   private:
    QList<QList<QVector3D>> m_hull;

    const float m_radius;
    const float m_height;
    const float m_angle;
    const float m_holeSize;
};

#endif  // MESH_H
