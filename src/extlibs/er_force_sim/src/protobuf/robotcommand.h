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

#ifndef ROBOTCOMMAND_H
#define ROBOTCOMMAND_H

#include <QtCore/QSharedPointer>

#include "extlibs/er_force_sim/src/protobuf/robot.pb.h"

//! @file robotcommand.h
//! @addtogroup protobuf
//! @{

//! Protobuf command wrapper with reference counting
typedef QSharedPointer<robot::Command> RobotCommand;

struct RobotCommandInfo
{
    unsigned int generation;
    unsigned int robotId;
    RobotCommand command;
};

//! @}

#endif  // ROBOTCOMMAND_H
