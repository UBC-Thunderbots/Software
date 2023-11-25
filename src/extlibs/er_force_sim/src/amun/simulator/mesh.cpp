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

#include "mesh.h"

#include <cmath>

using namespace camun::simulator;

/*!
 * \class Mesh
 * \ingroup simulator
 * \brief A 3D mesh
 */

Mesh::Mesh(float radius, float height, float angle, float holeSize, float boxHeight)
    : m_radius(radius), m_height(height), m_angle(angle), m_holeSize(holeSize)
{
    const float frontPlateLength = std::sin(angle / 2.0f) * radius;
    const float frontPlatePos    = radius * std::cos(angle / 2.0f);
    const float holePlatePos     = frontPlatePos - holeSize;
    const float outerAngle       = std::acos(holePlatePos / radius) * 2;
    const float angleDiff        = (outerAngle - angle) / 2.0f;
    const float halfOuterAngle   = outerAngle / 2.0f;
    const float outerAngleStart  = halfOuterAngle + M_PI_2;
    const float outerAngleStop   = 2.0f * M_PI - halfOuterAngle + M_PI_2;
    addRobotCover(20, outerAngleStart, outerAngleStop);

    // right pillar
    addRobotCover(5, outerAngleStart - angleDiff, outerAngleStart);
    m_hull.back().append(QVector3D(-frontPlateLength, holePlatePos, m_height / 2.0f));
    m_hull.back().append(QVector3D(-frontPlateLength, holePlatePos, -m_height / 2.0f));

    // left pillar
    addRobotCover(5, outerAngleStop, outerAngleStop + angleDiff);
    m_hull.back().append(QVector3D(frontPlateLength, holePlatePos, m_height / 2.0f));
    m_hull.back().append(QVector3D(frontPlateLength, holePlatePos, -m_height / 2.0f));

    // the remaining box
    QList<QVector3D> boxPart;
    boxPart.append(QVector3D(frontPlateLength, holePlatePos, m_height / 2.0f));
    boxPart.append(QVector3D(-frontPlateLength, holePlatePos, m_height / 2.0f));
    boxPart.append(
        QVector3D(frontPlateLength, holePlatePos, -m_height / 2.0f + boxHeight));
    boxPart.append(
        QVector3D(-frontPlateLength, holePlatePos, -m_height / 2.0f + boxHeight));
    boxPart.append(QVector3D(frontPlateLength, frontPlatePos, m_height / 2.0f));
    boxPart.append(QVector3D(-frontPlateLength, frontPlatePos, m_height / 2.0f));
    boxPart.append(
        QVector3D(frontPlateLength, frontPlatePos, -m_height / 2.0f + boxHeight));
    boxPart.append(
        QVector3D(-frontPlateLength, frontPlatePos, -m_height / 2.0f + boxHeight));
    m_hull.append(boxPart);
}

/*!
 * \brief Adds triangles for the outer hull
 * \param radius Radius of the robot
 * \param height Height of the robot
 * \param num Number of segments
 * \param angle Angle of the hull start
 * \param angleStep Step size in rad
 */
void Mesh::addRobotCover(uint num, float startAngle, float endAngle)
{
    QList<QVector3D> covers;
    float angle     = startAngle;
    float angleStep = (endAngle - startAngle) / num;
    for (uint i = 0; i <= num; i++)
    {
        covers.append(
            QVector3D(m_radius * cos(angle), m_radius * sin(angle), m_height / 2.0f));
        covers.append(
            QVector3D(m_radius * cos(angle), m_radius * sin(angle), -m_height / 2.0f));

        angle += angleStep;
    }
    m_hull.append(covers);
}
