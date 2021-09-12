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
 * \param teamInfo The team info to initialize
 */
void teamInfoSetDefault(SSL_Referee::TeamInfo *teamInfo)
{
    teamInfo->set_name("");
    teamInfo->set_score(0);
    teamInfo->set_red_cards(0);
    teamInfo->set_yellow_cards(0);
    teamInfo->set_timeouts(4);
    teamInfo->set_timeout_time(5 * 60 * 1000 * 1000);
    teamInfo->set_goalie(0);
    assert(teamInfo->IsInitialized());
}

SSL_Referee::Command commandFromGameState(amun::GameState::State state)
{
    switch (state)
    {
        case amun::GameState::Halt:
            return SSL_Referee::HALT;
        case amun::GameState::Stop:
            return SSL_Referee::STOP;
        case amun::GameState::Game:
            return SSL_Referee::FORCE_START;
        case amun::GameState::GameForce:
            return SSL_Referee::FORCE_START;
        case amun::GameState::KickoffYellowPrepare:
            return SSL_Referee::PREPARE_KICKOFF_YELLOW;
        case amun::GameState::KickoffYellow:
            return SSL_Referee::NORMAL_START;
        case amun::GameState::PenaltyYellowPrepare:
            return SSL_Referee::PREPARE_PENALTY_YELLOW;
        case amun::GameState::PenaltyYellow:
            return SSL_Referee::NORMAL_START;
        case amun::GameState::PenaltyYellowRunning:
            return SSL_Referee::NORMAL_START;
        case amun::GameState::DirectYellow:
            return SSL_Referee::DIRECT_FREE_YELLOW;
        case amun::GameState::IndirectYellow:
            return SSL_Referee::INDIRECT_FREE_YELLOW;
        case amun::GameState::BallPlacementYellow:
            return SSL_Referee::BALL_PLACEMENT_YELLOW;
        case amun::GameState::KickoffBluePrepare:
            return SSL_Referee::PREPARE_KICKOFF_BLUE;
        case amun::GameState::KickoffBlue:
            return SSL_Referee::NORMAL_START;
        case amun::GameState::PenaltyBluePrepare:
            return SSL_Referee::PREPARE_PENALTY_BLUE;
        case amun::GameState::PenaltyBlue:
            return SSL_Referee::NORMAL_START;
        case amun::GameState::PenaltyBlueRunning:
            return SSL_Referee::NORMAL_START;
        case amun::GameState::DirectBlue:
            return SSL_Referee::DIRECT_FREE_BLUE;
        case amun::GameState::IndirectBlue:
            return SSL_Referee::INDIRECT_FREE_BLUE;
        case amun::GameState::BallPlacementBlue:
            return SSL_Referee::BALL_PLACEMENT_BLUE;
        case amun::GameState::TimeoutYellow:
            return SSL_Referee::TIMEOUT_YELLOW;
        case amun::GameState::TimeoutBlue:
            return SSL_Referee::TIMEOUT_BLUE;
    }
    return SSL_Referee::HALT;  // should never be reached
}
