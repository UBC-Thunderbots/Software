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

#include "erroraggregator.h"

#include <QtCore/QList>

#include "simulator.h"

using namespace camun::simulator;

void ErrorAggregator::aggregate(SSLSimError error, ErrorSource source)
{
    m_data[source].push_back(std::move(error));
}

QList<SSLSimError> ErrorAggregator::getAggregates(ErrorSource source)
{
    QList<SSLSimError> &ref = m_data[source];
    QList<SSLSimError> out  = ref;
    ref.clear();
    return out;
}
