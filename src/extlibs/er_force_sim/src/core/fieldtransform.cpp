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

#include "fieldtransform.h"

#include <cmath>

FieldTransform::FieldTransform()
    : m_lastFlipped(false),
      m_hasTransform(false),
      m_transform({1, 0, 0, 1, 0, 0}),
      m_flipFactor(1.0f)
{
}

void FieldTransform::setFlip(bool flip)
{
    m_lastFlipped = flip;
    m_flipFactor  = flip ? -1.0f : 1.0f;
}

void FieldTransform::setTransform(const std::array<float, 6> &values)
{
    m_hasTransform = values != std::array<float, 6>({1, 0, 0, 1, 0, 0});
    m_transform    = values;
}

float FieldTransform::applyPosX(float x, float y) const
{
    return m_flipFactor * (m_transform[0] * x + m_transform[1] * y + m_transform[4]);
}

float FieldTransform::applyPosY(float x, float y) const
{
    return m_flipFactor * (m_transform[2] * x + m_transform[3] * y + m_transform[5]);
}

QPointF FieldTransform::applyPosition(const QPointF &pos) const
{
    return QPointF(applyPosX(pos.x(), pos.y()), applyPosY(pos.x(), pos.y()));
}

float FieldTransform::applySpeedX(float x, float y) const
{
    return m_flipFactor * (m_transform[0] * x + m_transform[1] * y);
}

float FieldTransform::applySpeedY(float x, float y) const
{
    return m_flipFactor * (m_transform[2] * x + m_transform[3] * y);
}

float FieldTransform::applyAngle(float angle) const
{
    if (m_lastFlipped && !m_hasTransform)
    {
        return angle + M_PI;
    }
    else if (!m_hasTransform)
    {
        return angle;
    }
    else
    {
        // only do this rather expensive calculation if a non regular transform is
        // set
        float x            = std::cos(angle);
        float y            = std::sin(angle);
        float xTransformed = applySpeedX(x, y);
        float yTransformed = applySpeedY(x, y);
        return std::atan2(yTransformed, xTransformed);
    }
}

float FieldTransform::applyInverseX(float x, float y) const
{
    x *= m_flipFactor;
    y *= m_flipFactor;
    x -= m_transform[4];
    y -= m_transform[5];
    float invDet = m_transform[0] * m_transform[3] - m_transform[1] * m_transform[2];
    return invDet * (m_transform[3] * x - m_transform[1] * y);
}

float FieldTransform::applyInverseY(float x, float y) const
{
    x *= m_flipFactor;
    y *= m_flipFactor;
    x -= m_transform[4];
    y -= m_transform[5];
    float invDet = m_transform[0] * m_transform[3] - m_transform[1] * m_transform[2];
    return invDet * (-m_transform[2] * x + m_transform[0] * y);
}

QPointF FieldTransform::applyInversePosition(const QPointF &pos) const
{
    return QPointF(applyInverseX(pos.x(), pos.y()), applyInverseY(pos.x(), pos.y()));
}
