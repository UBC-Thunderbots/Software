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

#ifndef TIMER_H
#define TIMER_H

#include <QtCore/QObject>
#include <QtCore/QtGlobal>

class Timer : public QObject
{
    Q_OBJECT

   public:
    Timer();

    double scaling() const
    {
        return m_scaling;
    }
    void setScaling(double scaling);
    void reset();
    qint64 currentTime() const;
    void setTime(qint64 time, double scaling);

   signals:
    void scalingChanged(float scaling);

   public:
    static qint64 systemTime();

   private:
    double m_scaling;
    qint64 m_start;
    qint64 m_offset;
};

#endif  // TIMER_H
