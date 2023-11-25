/***************************************************************************
 *   Copyright 2020 Andreas Wendler                                        *
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

#ifndef PROTOBUFFILEREADER_H
#define PROTOBUFFILEREADER_H

#include <google/protobuf/message.h>

#include <QtCore/QDataStream>
#include <QtCore/QFile>

class ProtobufFileReader : public QObject
{
   public:
    ProtobufFileReader();

    bool open(QString filename, QString filePrefix);

    // returns true if the message has be sucessfully parsed
    bool readNext(google::protobuf::Message &message);

   private:
    QFile m_file;
    QDataStream m_stream;
};

#endif  // PROTOBUFFILEREADER_H
