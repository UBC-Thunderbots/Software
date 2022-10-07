/***************************************************************************
 *   Copyright 2015 Michael Eischer                                        *
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

#include "ssl_referee.h"

/*!
 * \brief Initializes team info with default values
 * \param team_info The team info to initialize
 */
void teamInfoSetDefault(SSLProto::Referee::TeamInfo *team_info)
{
    team_info->set_name("");
    team_info->set_score(0);
    team_info->set_red_cards(0);
    team_info->set_yellow_cards(0);
    team_info->set_timeouts(4);
    team_info->set_timeout_time(5 * 60 * 1000 * 1000);
    team_info->set_goalkeeper(0);
    assert(team_info->IsInitialized());
}
