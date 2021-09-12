/***************************************************************************
 *   Copyright 2020 Andreas Wendler                                        *
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

#ifndef FIELDTRANSFORM_H
#define FIELDTRANSFORM_H

#include <QtCore/QPointF>
#include <array>

class FieldTransform
{
   public:
    FieldTransform();
    void setFlip(bool flip);
    void setTransform(const std::array<float, 6> &values);
    float applyPosX(float x, float y) const;
    float applyPosY(float x, float y) const;
    QPointF applyPosition(const QPointF &pos) const;
    float applySpeedX(float x, float y) const;
    float applySpeedY(float x, float y) const;
    float applyAngle(float angle) const;
    float applyInverseX(float x, float y) const;
    float applyInverseY(float x, float y) const;
    QPointF applyInversePosition(const QPointF &pos) const;

   private:
    bool m_lastFlipped;
    bool m_hasTransform;
    std::array<float, 6> m_transform;
    float m_flipFactor;
};

#endif  // FIELDTRANSFORM_H
