/***************************************************************************
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

#include "fastsimulator.h"

#include <algorithm>

#include "simulator.h"

bool FastSimulator::goWithCallback(camun::simulator::Simulator *sim, Timer *t,
                                   qint64 targetTime,
                                   const std::function<void(void)> &callback)
{
    if (t->scaling() != 0)
        return false;
    callback();
    const qint64 maxSimulationStep = 1e9 / 200;
    qint64 now                     = t->currentTime();
    qint64 lastCallbackTime        = now;
    while (now != targetTime)
    {
        qint64 nextTime = std::min(now + maxSimulationStep, targetTime);
        t->setTime(nextTime, 0);
        sim->process();
        now = t->currentTime();
        if (now - lastCallbackTime >= 1e7)
        {
            callback();
            lastCallbackTime = now;
        }
    }
    return true;
}
