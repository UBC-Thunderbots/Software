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

#ifndef PB_SSLSIM_H
#define PB_SSLSIM_H

#include <QtCore/QSharedPointer>

#include "extlibs/er_force_sim/src/protobuf/ssl_simulation_error.pb.h"
#include "extlibs/er_force_sim/src/protobuf/ssl_simulation_robot_control.pb.h"

typedef QSharedPointer<sslsim::RobotControl> SSLSimRobotControl;
typedef QSharedPointer<sslsim::SimulatorError> SSLSimError;
#endif
