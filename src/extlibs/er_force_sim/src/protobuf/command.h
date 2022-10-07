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

#ifndef COMMAND_H
#define COMMAND_H

#include <QtCore/QSharedPointer>

#include "extlibs/er_force_sim/src/protobuf/command.pb.h"

//! @file command.h
//! @addtogroup protobuf
//! @{

void simulatorSetupSetDefault(amun::SimulatorSetup &setup);

// position is in meters in our coordinate system
SSL_GeometryCameraCalibration createDefaultCamera(int camera_id, float x, float y,
                                                  float z);

//! @}

#endif  // COMMAND_H
