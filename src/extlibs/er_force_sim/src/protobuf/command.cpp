/***************************************************************************
 *   Copyright 2021 Andreas Wendler                                        *
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

#include "command.h"

#include "geometry.h"

SSL_GeometryCameraCalibration createDefaultCamera(int camera_id, float x, float y,
                                                  float z)
{
    SSL_GeometryCameraCalibration calibration;

    calibration.set_camera_id(camera_id);
    // DUMMY VALUES
    calibration.set_distortion(0.2);
    calibration.set_focal_length(390);
    calibration.set_principal_point_x(300);
    calibration.set_principal_point_y(300);
    calibration.set_q0(0.7);
    calibration.set_q1(0.7);
    calibration.set_q2(0.7);
    calibration.set_q3(0.7);
    calibration.set_tx(0);
    calibration.set_ty(0);
    calibration.set_tz(3500);

    calibration.set_derived_camera_world_tx(y * 1000);
    calibration.set_derived_camera_world_ty(-x * 1000);
    calibration.set_derived_camera_world_tz(z * 1000);

    return calibration;
}

void simulatorSetupSetDefault(amun::SimulatorSetup &setup)
{
    geometrySetDefault(setup.mutable_geometry(), true);
    setup.add_camera_setup()->CopyFrom(createDefaultCamera(0, 0.0f, 0.0f, 4.0f));
}
