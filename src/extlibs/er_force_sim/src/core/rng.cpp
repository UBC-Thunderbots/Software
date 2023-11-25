/***************************************************************************
 *   Copyright 2015 Philipp Nordhus                                        *
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

#include "rng.h"

#include <sys/time.h>

#include <cmath>
#include <cstdlib>

/*!
 * \class RNG
 * \ingroup core
 * \brief Pseudorandom number generator
 *
 * http://www.iro.umontreal.ca/~lecuyer/myftp/papers/tausme.ps
 */

#define LCG(n) ((69069 * n) & 0xffffffffU)
#define MASK 0xffffffffU

static inline uint32_t TAUSWORTHE(uint32_t s, int a, int b, uint32_t c, int d)
{
    return (((s & c) << d) & MASK) ^ (((s << a) & MASK) ^ s) >> b;
}

/*!
 * \brief Create an RNG instance
 * \param seed RNG seed
 */
RNG::RNG(uint32_t s)
{
    if (s == 0)
    {
        timeval tv;
        gettimeofday(&tv, NULL);
        s = tv.tv_sec * 1000 * 1000 + tv.tv_usec;
    }

    seed(s);
}

void RNG::seed(uint32_t s)
{
    if (s == 0)
    {
        // The RNG breaks if it is seeded with 0, replace the seed with a legitimate
        // value
        s = -1;
    }

    m_s1 = LCG(s);
    m_s2 = LCG(m_s1);
    m_s3 = LCG(m_s2);

    for (int i = 0; i < 6; i++)
    {
        uniformInt();
    }
}

/*!
 * \brief Generate a random integer in the range [0, 2^32-1]
 * \return A random number drawn from a uniform distribution [0, 2^32-1]
 */
uint32_t RNG::uniformInt()
{
    m_s1 = TAUSWORTHE(m_s1, 13, 19, 4294967294U, 12);
    m_s2 = TAUSWORTHE(m_s2, 2, 25, 4294967288U, 4);
    m_s3 = TAUSWORTHE(m_s3, 3, 11, 4294967280U, 17);

    return (m_s1 ^ m_s2 ^ m_s3);
}

/*!
 * \brief Generate a random floating point number in the range (0, 1]
 * \return A random number drawn from a uniform distribution (0, 1]
 */
double RNG::uniformPositive()
{
    double r;

    do
    {
        r = uniform();
    } while (r == 0.0);

    return r;
}

/*!
 * \brief Generate a vector with two independent random components drawn from a
 * normal distribution \param sigma Standard deviation \param mean Expected
 * value \return A random vector drawn from a normal distribution
 */
ErForceVector RNG::normalVector(double sigma, double mean)
{
    double u;
    double v;
    double s;

    do
    {
        u = -1.0 + 2.0 * uniformPositive();
        v = -1.0 + 2.0 * uniformPositive();

        s = u * u + v * v;
    } while (s == 0.0 || s >= 1.0);

    // Box-Muller transform (polar)
    const double tmp = sigma * std::sqrt(-2.0 * std::log(s) / s);

    return ErForceVector(tmp * u + mean, tmp * v + mean);
}
