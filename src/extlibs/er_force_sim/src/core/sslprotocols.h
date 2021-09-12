/***************************************************************************
 *   Copyright 2021 Paul Bergmann                                          *
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

#ifndef SSLPROTOCOLS_H
#define SSLPROTOCOLS_H

#include <cstdint>

// See https://ssl.robocup.org/league-software/ "Standard Network Parameters"

/* The game controller publishes referee commands using UDP multicast on this
 * address:port
 */
constexpr const char* SSL_GAME_CONTROLLER_ADDRESS = "224.5.23.1";
constexpr const uint16_t SSL_GAME_CONTROLLER_PORT = 10003;

/* SSL vision publishes vision detections using UDP multicast on this
 * address:port
 */
constexpr const char* SSL_VISION_ADDRESS = "224.5.23.2";
constexpr const uint16_t SSL_VISION_PORT = 10006;

/* A simulator will publish vision data using this port instead of
 * `SSL_VISION_PORT`
 */
constexpr const uint16_t SSL_SIMULATED_VISION_PORT = 10020;

// The game controller will listen for autoref messages on this port
constexpr const uint16_t SSL_AUTOREF_TO_GC_PORT = 10007;

/* The game controller will listen for team messages (e.g. bot substitution) on
 * this port
 */
constexpr const uint16_t SSL_TEAM_TO_GC_PORT = 10008;

/* A SSL vision tracker (e.g an autoref) may publish tracked vision data on
 * this address:port
 */
constexpr const char* SSL_VISION_TRACKER_ADDRESS = "224.5.23.2";
constexpr const uint16_t SSL_VISION_TRACKER_PORT = 10010;

/* A simulator will listen for simulation control commands (e.g ball
 * teleportation) on this port
 */
constexpr const uint16_t SSL_SIMULATION_CONTROL_PORT = 10300;

/* A simulator will listen for robot control commands for the blue team on this
 * port
 */
constexpr const uint16_t SSL_SIMULATION_CONTROL_BLUE_PORT = 10301;

/* A simulator will listen for robot control commands for the yellow team on
 * this port
 */
constexpr const uint16_t SSL_SIMULATION_CONTROL_YELLOW_PORT = 10302;

constexpr const uint16_t SSL_MIXED_TEAM_PORT = 10012;

#endif  // SSLPROTOCOLS_H
