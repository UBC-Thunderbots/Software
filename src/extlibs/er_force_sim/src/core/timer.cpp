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

#include "timer.h"

#if _POSIX_TIMERS > 0
#define WITH_POSIX_TIMERS
#endif

#ifdef WITH_POSIX_TIMERS
#include <time.h>
#else
#include <sys/time.h>
#ifdef Q_OS_WIN
#include <windows.h>
#endif  // Q_OS_WIN
#endif

/*!
 * \class Timer
 * \ingroup core
 * \brief High precision timer
 */

/*!
 * \brief Creates a new timer object
 */
Timer::Timer()
{
    reset();
}

/*!
 * \brief Sets time scaling. Time is guaranteed to be continuous
 * \param scaling New scaling factor
 */
void Timer::setScaling(double scaling)
{
    Q_ASSERT(scaling >= 0);
    setTime(currentTime(), scaling);
    emit scalingChanged(scaling);
}

/*!
 * \brief Reset timer to current time and reset Scaling
 */
void Timer::reset()
{
    setTime(systemTime(), 1.0);
}

/*!
 * \brief Query internal time
 * \return The internal time in nanoseconds
 */
qint64 Timer::currentTime() const
{
    const qint64 sys = systemTime();
    return m_offset + (qint64)((sys - m_start) * m_scaling);
}

/*!
 * \brief Set internal time and scaling
 * \param time New internal time
 * \param scaling New scaling factor
 */
void Timer::setTime(qint64 time, double scaling)
{
    Q_ASSERT(scaling >= 0);
    m_offset  = time;
    m_start   = systemTime();
    m_scaling = scaling;
}

/*!
 * \brief Query system time
 * \return The current system time in nanoseconds
 */
qint64 Timer::systemTime()
{
#ifdef WITH_POSIX_TIMERS
    timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
        return 0;

    return qint64(ts.tv_sec) * 1000000000LL + qint64(ts.tv_nsec);
#else  // WITH_POSIX_TIMERS
#ifdef Q_OS_WIN
    static bool isInitialized = false;

    static quint64 timerFrequency;
    static quint64 startTick;
    static quint64 startTime;

    if (!isInitialized)
    {
        isInitialized  = true;
        timerFrequency = 0;  // !!! disable QueryPerformanceCounter for initialization
        startTime      = Timer::systemTime();  // get time via gettimeofday

        // the timing provided by this code is far from optimal, but it should do
        // as the time deltas are what we're interested in.
        LARGE_INTEGER freq;
        // set timerFrequency to zero if QueryPerformanceCounter can't be used
        if (!QueryPerformanceFrequency(&freq))
        {
            timerFrequency = 0;
        }
        else
        {
            timerFrequency = freq.QuadPart;
        }
        if (timerFrequency > 0)
        {
            LARGE_INTEGER ticks;
            if (QueryPerformanceCounter(&ticks))
            {
                startTick = ticks.QuadPart;
            }
            else
            {
                timerFrequency = 0;
            }
        }
    }

    if (timerFrequency > 0)
    {
        LARGE_INTEGER ticks;
        if (QueryPerformanceCounter(&ticks))
        {
            return (ticks.QuadPart - startTick) * 1000000000LL / timerFrequency +
                   startTime;
        }
    }
#endif  // Q_OS_WIN

    timeval tv;
    if (gettimeofday(&tv, NULL) != 0)
    {
        return 0;
    }

    return qint64(tv.tv_sec) * 1000000000LL + qint64(tv.tv_usec) * 1000LL;
#endif  // WITH_POSIX_TIMERS
}
