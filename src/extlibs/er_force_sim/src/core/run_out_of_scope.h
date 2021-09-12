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

#ifndef RUN_WHEN_OUT_OF_SCOPE

namespace core
{
    namespace internal
    {
        template <class T>
        class Runner
        {
           public:
            Runner(T lambda) : m_lambda(lambda) {}
            ~Runner()
            {
                m_lambda();
            }

           private:
            T m_lambda;
        };

        template <class T>
        Runner<T> make_runner(T t)
        {
            return Runner<T>{t};
        }
    }  // namespace internal
}  // namespace core

#define RUN_WHEN_OUT_OF_SCOPE(X)                                                         \
    auto r                                                                               \
    {                                                                                    \
        core::internal::make_runner([&]() X)                                             \
    }
#endif
