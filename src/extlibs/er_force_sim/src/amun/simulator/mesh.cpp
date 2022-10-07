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

Mesh::Mesh(float radius, float height, float angle, float hole_size, float box_height)
    : m_radius(radius), m_height(height), m_angle(angle), m_hole_size(hole_size)
{
    const float front_plate_length = std::sin(angle / 2.0f) * radius;
    const float front_plate_pos    = radius * std::cos(angle / 2.0f);
    const float hole_plate_pos     = front_plate_pos - hole_size;
    const float outer_angle       = std::acos(hole_plate_pos / radius) * 2;
    const float angle_diff        = (outer_angle - angle) / 2.0f;
    const float half_outer_angle   = outer_angle / 2.0f;
    const float outer_angle_start  = half_outer_angle + M_PI_2;
    const float outer_angle_stop   = 2.0f * M_PI - half_outer_angle + M_PI_2;
    addRobotCover(20, outer_angle_start, outer_angle_stop);

    // right pillar
    addRobotCover(5, outer_angle_start - angle_diff, outer_angle_start);
    m_hull.back().append(QVector3D(-front_plate_length, hole_plate_pos, m_height / 2.0f));
    m_hull.back().append(QVector3D(-front_plate_length, hole_plate_pos, -m_height / 2.0f));

    // left pillar
    addRobotCover(5, outer_angle_stop, outer_angle_stop + angle_diff);
    m_hull.back().append(QVector3D(front_plate_length, hole_plate_pos, m_height / 2.0f));
    m_hull.back().append(QVector3D(front_plate_length, hole_plate_pos, -m_height / 2.0f));

    // the remaining box
    QList<QVector3D> boxPart;
    boxPart.append(QVector3D(front_plate_length, hole_plate_pos, m_height / 2.0f));
    boxPart.append(QVector3D(-front_plate_length, hole_plate_pos, m_height / 2.0f));
    boxPart.append(
        QVector3D(front_plate_length, hole_plate_pos, -m_height / 2.0f + box_height));
    boxPart.append(
        QVector3D(-front_plate_length, hole_plate_pos, -m_height / 2.0f + box_height));
    boxPart.append(QVector3D(front_plate_length, front_plate_pos, m_height / 2.0f));
    boxPart.append(QVector3D(-front_plate_length, front_plate_pos, m_height / 2.0f));
    boxPart.append(
        QVector3D(front_plate_length, front_plate_pos, -m_height / 2.0f + box_height));
    boxPart.append(
        QVector3D(-front_plate_length, front_plate_pos, -m_height / 2.0f + box_height));
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
void Mesh::addRobotCover(uint num, float start_angle, float end_angle)
{
    QList<QVector3D> covers;
    float angle      = start_angle;
    float angle_step = (end_angle - start_angle) / num;
    for (uint i = 0; i <= num; i++)
    {
        covers.append(
            QVector3D(m_radius * cos(angle), m_radius * sin(angle), m_height / 2.0f));
        covers.append(
            QVector3D(m_radius * cos(angle), m_radius * sin(angle), -m_height / 2.0f));

        angle += angle_step;
    }
    m_hull.append(covers);
}
