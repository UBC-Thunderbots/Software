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

#ifndef STATUS_H
#define STATUS_H

#include <QtCore/QSharedPointer>

#include "extlibs/er_force_sim/src/protobuf/status.pb.h"

//! @file status.h
//! @addtogroup protobuf
//! @{

class Status
{
   public:
    Status() {}

    Status(amun::Status *status)
    {
        m_status = QSharedPointer<amun::Status>(status);
    }

    void clear()
    {
        m_status.clear();
        m_arena.clear();
    }

    bool isNull() const
    {
        return m_arena.isNull() && m_status.isNull();
    }

    amun::Status &operator*() const
    {
        if (m_arena.isNull())
            return *m_status;
        else
            return *m_arenaStatus;
    }

    amun::Status *operator->() const
    {
        if (m_arena.isNull())
            return &(*m_status);
        else
            return m_arenaStatus;
    }

    static Status createArena()
    {
        google::protobuf::ArenaOptions options;
        options.initial_block_size     = 512;
        options.max_block_size         = 32 * 1024;
        google::protobuf::Arena *arena = new google::protobuf::Arena(options);
        amun::Status *s = google::protobuf::Arena::CreateMessage<amun::Status>(arena);
        return Status(s, arena);
    }

   private:
    Status(amun::Status *status, google::protobuf::Arena *arena)
    {
        m_arenaStatus = status;
        m_arena       = QSharedPointer<google::protobuf::Arena>(arena);
    }

    QSharedPointer<amun::Status> m_status;
    amun::Status *m_arenaStatus = nullptr;
    QSharedPointer<google::protobuf::Arena> m_arena;
};

//! @}

#endif  // STATUS_H
