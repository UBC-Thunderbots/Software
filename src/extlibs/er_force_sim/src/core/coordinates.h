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
        float detection_x, detection_y;
        core::internal::getPos(from, detection_x, detection_y);
        float x = -detection_y / 1000.0f;
        float y = detection_x / 1000.0f;
        core::internal::setPos(to, x, y);
    }

    template <class F, class T>
    void toVision(const F& from, T& to)
    {
        float x, y;
        core::internal::getPos(from, x, y);
        float vision_x, vision_y;
        vision_x = y * 1000.f;
        vision_y = -x * 1000.f;
        core::internal::setPos(to, vision_x, vision_y);
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

    inline float toVisionRotation(float intern_rotation)
    {
        return intern_rotation - M_PI_2;
    }

    inline float chipVelFromChipDistance(float distance)
    {
        const float angle     = 45.f / 180 * M_PI;
        const float dir_floor = std::cos(angle);
        const float dir_up    = std::sin(angle);
        const float gravity   = 9.81;
        // airtime = 2 * (shoot_speed * dir_up) / g
        // targetDist = shoot_speed * dir_floor * airtime
        // => targetDist = shoot_speed * dir_floor * (2 * shoot_speed * dir_up) / g = 2 *
        // shoot_speed**2 * dir_floor * dir_up / g
        const float shoot_speed =
            std::sqrt(distance * gravity / (2 * std::abs(dir_up * dir_floor)));
        return shoot_speed;
    }

    inline float chipDistanceFromChipVel(float velocity)
    {
        const float angle     = 45.f / 180 * M_PI;
        const float dir_floor = std::cos(angle);
        const float dir_up    = std::sin(angle);
        const float gravity   = 9.81;
        // airtime = 2 * (shootSpeed * dir_up) / g
        // target_dist = shootSpeed * dir_floor * airtime
        // => target_dist = shootSpeed * dir_floor * (2 * shootSpeed * dir_up) / g = 2 *
        // shootSpeed**2 * dir_floor * dir_up / g
        const float target_dist = 2 * velocity * velocity * dir_floor * dir_up / gravity;

        return target_dist;
    }

}  // namespace coordinates

#endif
