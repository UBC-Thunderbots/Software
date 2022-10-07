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

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "extlibs/er_force_sim/src/protobuf/world.pb.h"
#include "proto/ssl_vision_wrapper.pb.h"

void geometrySetDefault(world::Geometry *geometry, bool use_quad_field = true);

void convertFromSSlGeometry(const SSLProto::SSL_GeometryFieldSize &g,
                            world::Geometry &out_geometry);
void convertToSSlGeometry(const world::Geometry &g,
                          SSLProto::SSL_GeometryFieldSize *out_geometry);

#endif  // GEOMETRY_H
