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

#ifndef CORE_COORDINATES
#define CORE_COORDINATES

#include <type_traits>

#include "vector.h"

namespace core
{
    namespace internal
    {
        // for pairs
        template <class T>
        auto set(T& t, float x, float y) -> decltype(t.first = x, void())
        {
            t.first  = x;
            t.second = y;
        }

        // for core vectors
        template <class T>
        auto set(T& t, float x, float y) -> decltype(t.x = x, void())
        {
            t.x = x;
            t.y = y;
        }

        // for bullet vectors
        template <class T>
        auto set(T& t, float x, float y) -> decltype(t.setX(x), void())
        {
            t.setX(x);
            t.setY(y);
        }

        // for protobuf messages
        template <class T>
        auto setPos(T& t, float x, float y) -> decltype(t.set_x(x), void())
        {
            t.set_x(x);
            t.set_y(y);
        }

        template <class T>
        auto setPos(T& t, float x, float y) -> decltype(set(t, x, y), void())
        {
            set(t, x, y);
        }

        template <class T>
        auto setVel(T& t, float x, float y) -> decltype(set(t, x, y), void())
        {
            set(t, x, y);
        }

        template <class T>
        auto setVel(T& t, float x, float y) -> decltype(t.set_v_x(x), void())
        {
            t.set_v_x(x);
            t.set_v_y(y);
        }

        template <class T>
        auto setVel(T& t, float x, float y) -> decltype(t.set_vx(x), void())
        {
            t.set_vx(x);
            t.set_vy(y);
        }

        template <class T>
        auto get(const T& t, float& x, float& y) -> decltype(x = t.first, void())
        {
            x = t.first;
            y = t.second;
        }


        template <class T>
        auto get(const T& t, float& x, float& y) -> decltype(x = t.x, void())
        {
            x = t.x;
            y = t.y;
        }


        // for our bullet vectors and some messages
        template <class T>
        auto getPos(const T& t, float& x, float& y) -> decltype(x = t.x(), void())
        {
            x = t.x();
            y = t.y();
        }

        template <class T>
        auto getPos(const T& t, float& x, float& y) -> decltype(get(t, x, y))
        {
            get(t, x, y);
        }

        template <class T>
        auto getVel(const T& t, float& x, float& y) -> decltype(get(t, x, y))
        {
            get(t, x, y);
        }

        template <class T>
        auto getVel(const T& t, float& x, float& y) -> decltype(x = t.v_x(), void())
        {
            x = t.v_x();
            y = t.v_y();
        }

        template <class T>
        auto getVel(const T& t, float& x, float& y) -> decltype(x = t.vx(), void())
        {
            x = t.vx();
            y = t.vy();
        }
    }  // namespace internal
}  // namespace core


namespace coordinates
{
    template <class F, class T>
    void fromVision(const F& from, T& to)
    {
        float detectionX, detectionY;
        core::internal::getPos(from, detectionX, detectionY);
        float x = -detectionY / 1000.0f;
        float y = detectionX / 1000.0f;
        core::internal::setPos(to, x, y);
    }

    template <class F, class T>
    void toVision(const F& from, T& to)
    {
        float x, y;
        core::internal::getPos(from, x, y);
        float visionX, visionY;
        visionX = y * 1000.f;
        visionY = -x * 1000.f;
        core::internal::setPos(to, visionX, visionY);
    }

    template <class F, class T>
    void fromVisionVelocity(const F& from, T& to)
    {
        std::pair<float, float> vision;
        std::pair<float, float> result;
        core::internal::getVel(from, vision.first, vision.second);
        fromVision(vision, result);
        core::internal::setVel(to, result.first, result.second);
    }

    template <class F, class T>
    void toVisionVelocity(const F& from, T& to)
    {
        std::pair<float, float> vision;
        std::pair<float, float> result;
        core::internal::getVel(from, vision.first, vision.second);
        toVision(vision, result);
        core::internal::setVel(to, result.first, result.second);
    }

    inline float fromVisionRotation(float vision)
    {
        return vision + M_PI_2;
    }

    inline float toVisionRotation(float internRotation)
    {
        return internRotation - M_PI_2;
    }

    inline float chipVelFromChipDistance(float distance)
    {
        const float angle    = 45.f / 180 * M_PI;
        const float dirFloor = std::cos(angle);
        const float dirUp    = std::sin(angle);
        const float gravity  = 9.81;
        // airtime = 2 * (shootSpeed * dirUp) / g
        // targetDist = shootSpeed * dirFloor * airtime
        // => targetDist = shootSpeed * dirFloor * (2 * shootSpeed * dirUp) / g = 2 *
        // shootSpeed**2 * dirFloor * dirUp / g
        const float shootSpeed =
            std::sqrt(distance * gravity / (2 * std::abs(dirUp * dirFloor)));
        return shootSpeed;
    }

    inline float chipDistanceFromChipVel(float velocity)
    {
        const float angle    = 45.f / 180 * M_PI;
        const float dirFloor = std::cos(angle);
        const float dirUp    = std::sin(angle);
        const float gravity  = 9.81;
        // airtime = 2 * (shootSpeed * dirUp) / g
        // targetDist = shootSpeed * dirFloor * airtime
        // => targetDist = shootSpeed * dirFloor * (2 * shootSpeed * dirUp) / g = 2 *
        // shootSpeed**2 * dirFloor * dirUp / g
        const float targetDist = 2 * velocity * velocity * dirFloor * dirUp / gravity;

        return targetDist;
    }

}  // namespace coordinates

#endif
