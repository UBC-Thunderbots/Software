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

#ifndef FASTSIMULATOR_H
#define FASTSIMULATOR_H

#include <QtCore/QtGlobal>
#include <functional>

#include "extlibs/er_force_sim/src/core/timer.h"

namespace camun
{
    namespace simulator
    {
        class Simulator;
    }
}  // namespace camun


namespace FastSimulator
{
    // calls the callback every 10 ms simulation time
    // Warning: the state of the callback timing will not be preserved between different
    // calls to this function. Instead, the callback will be called at the beginning and
    // every 10 ms after that.
    bool goWithCallback(camun::simulator::Simulator* sim, Timer* t, qint64 targetTime,
                        const std::function<void(void)>& callback);

    inline bool goDeltaCallback(camun::simulator::Simulator* sim, Timer* t, qint64 delta,
                                const std::function<void(void)>& cb)
    {
        return goWithCallback(sim, t, t->currentTime() + delta, cb);
    }

    // runs the simulation single-threaded as fast as your CPU will allow,
    // only possible for simulators created with useManualTrigger = true, and with
    // timer.scaling = 0
    // @return: returns false if one of these preconditions was violated and the process
    // was aborted, true otherwise.
    inline bool goToTime(camun::simulator::Simulator* sim, Timer* t, qint64 targetTime)
    {
        return goWithCallback(sim, t, targetTime, []() {});
    }

    inline bool goDelta(camun::simulator::Simulator* sim, Timer* t, qint64 delta)
    {
        return goToTime(sim, t, t->currentTime() + delta);
    }

}  // namespace FastSimulator
#endif
