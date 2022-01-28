/***************************************************************************
 *   Copyright 2018 Andreas Wendler                                        *
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

#include "robot.h"

void robotSetDefault(robot::Specs *specs)
{
    specs->set_radius(0.09f);
    specs->set_height(0.15f);
    specs->set_generation(3);
    specs->set_year(2014);
    specs->set_id(0);
    specs->set_type(robot::Specs_GenerationType_Regular);
    specs->set_mass(1.5f);
    specs->set_angle(0.982f);
    specs->set_v_max(3.5f);
    specs->set_omega_max(6);
    specs->set_shot_linear_max(6.5f);
    specs->set_shot_chip_max(3);
    specs->set_dribbler_width(0.07f);
    specs->set_ir_param(40);
    specs->set_shoot_radius(0.067f);
    specs->set_dribbler_height(0.04f);
    robot::LimitParameters *acceleration = specs->mutable_acceleration();
    acceleration->set_a_speedup_f_max(7);
    acceleration->set_a_speedup_s_max(6);
    acceleration->set_a_speedup_phi_max(60);
    acceleration->set_a_brake_f_max(7);
    acceleration->set_a_brake_s_max(6);
    acceleration->set_a_brake_phi_max(60);

    robot::LimitParameters *strategy = specs->mutable_strategy();
    strategy->set_a_speedup_f_max(4);
    strategy->set_a_speedup_s_max(3);
    strategy->set_a_speedup_phi_max(45);
    strategy->set_a_brake_f_max(3);
    strategy->set_a_brake_s_max(3);
    strategy->set_a_brake_phi_max(45);
}
