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

#ifndef PLOT_H
#define PLOT_H

#include <QtCore/QString>
#include <QtCore/QVariant>
#include <QtCore/QVector>
#include <QtGui/QColor>

class Plot : public QObject
{
    Q_OBJECT
   public:
    explicit Plot(const QString &name, QObject *parent = 0);

   public:
    void addPoint(float time, float value);
    void plot(const QColor &color) const;
    void mergeFrom(const Plot *p);
    void clearData();
    static int bufferSize()
    {
        return BUFFER_SIZE;
    }

    const QString &name() const
    {
        return m_name;
    }
    float time() const
    {
        return m_time;
    }

   private:
    QString m_name;
    // used as ringbuffer
    QVector<float> m_data;
    int m_pos;
    int m_count;
    float m_time;
    float m_maxTime;
    static const int BUFFER_SIZE = 6000;
};

Q_DECLARE_METATYPE(Plot *)

#endif  // PLOT_H
