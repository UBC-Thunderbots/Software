/****************************************************************************
 *   Copyright 2021 Tobias Heineken                                        *
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

#ifndef SIM_AGGREGATOR_H
#define SIM_AGGREGATOR_H
#include <QtCore/QMap>
#include <QtCore/QObject>

#include "extlibs/er_force_sim/src/protobuf/sslsim.h"

namespace camun
{
    namespace simulator
    {
        enum class ErrorSource;
        class ErrorAggregator : public QObject
        {
            Q_OBJECT
           public:
            ErrorAggregator(QObject* parent) : QObject(parent) {}

           public slots:
            void aggregate(SSLSimError eror, ErrorSource e);

           public:
            QList<SSLSimError> getAggregates(ErrorSource e);

           private:
            QMap<ErrorSource, QList<SSLSimError>> m_data;
        };
    }  // namespace simulator
}  // namespace camun
#endif
