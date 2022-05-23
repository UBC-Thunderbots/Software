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

#include "plot.h"

#include <QtOpenGL/QGLWidget>
#include <limits>

Plot::Plot(const QString &name, QObject *parent)
    : QObject(parent),  // use QObject for garbage collection
      m_name(name),
      m_pos(0),
      m_count(0),
      m_time(0),
      m_maxTime(-std::numeric_limits<float>::max())
{
    // ringbuffer with 6000 entrys as time, value pair
    m_data.resize(BUFFER_SIZE * 2);
}

void Plot::addPoint(float time, float value)
{
    if (m_pos == m_data.size())
    {  // wrap around
        m_pos = 0;
    }

    m_time = time;  // remember last update
    if (time < m_maxTime)
    {
        // make sure all data points are always ordered increasing by time
        int arrayStart = (m_pos + m_data.size() - m_count) % m_data.size();
        // binary search
        int max = m_count / 2;
        int min = 0;
        while (max - min > 0)
        {
            int pos     = (max + min) / 2;
            float value = m_data[(arrayStart + pos * 2) % m_data.size()];
            if (value == time)
            {
                min = pos + 1;
                break;
            }
            else if (value > time)
            {
                max = pos - 1;
            }
            else
            {
                min = pos + 1;
            }
        }
        min       = std::max(min, 1);
        m_pos     = (arrayStart + (min - 1) * 2) % m_data.size();
        m_count   = (min - 1) * 2;
        m_maxTime = time;
    }
    m_maxTime       = std::max(m_maxTime, time);
    m_data[m_pos++] = time;  // save value pair
    m_data[m_pos++] = value;

    // increase count while the buffer isn't full yet
    if (m_count < m_data.size())
    {
        m_count += 2;
    }
}

void Plot::plot(const QColor &color) const
{
    glLineWidth(3.0f);
    glColor3f(color.redF(), color.greenF(), color.blueF());
    glVertexPointer(2, GL_FLOAT, 0, m_data.data());
    glEnableClientState(GL_VERTEX_ARRAY);
    if (m_count > m_pos)
    {
        glDrawArrays(GL_LINE_STRIP, (m_data.size() - (m_count - m_pos)) / 2,
                     (m_count - m_pos) / 2);
    }
    glDrawArrays(GL_LINE_STRIP, std::max(m_pos - m_count, 0) / 2,
                 (m_pos - std::max(m_pos - m_count, 0)) / 2);
    glLineWidth(1.0f);
}

void Plot::mergeFrom(const Plot *p)
{
    // start after the latest value, until the last value
    for (int i = p->m_pos; i < p->m_count; i += 2)
    {
        addPoint(p->m_data[i], p->m_data[i + 1]);
    }
    // remaining value until the latest value
    for (int i = 0; i < p->m_pos; i += 2)
    {
        addPoint(p->m_data[i], p->m_data[i + 1]);
    }
}

void Plot::clearData()
{
    m_pos   = 0;
    m_count = 0;
}
