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

#include "protobuffilereader.h"

ProtobufFileReader::ProtobufFileReader() : m_stream(&m_file) {}

bool ProtobufFileReader::open(QString filename, QString file_prefix)
{
    m_file.setFileName(filename);
    if (!m_file.open(QIODevice::ReadOnly))
    {
        return false;
    }

    // ensure compatibility across qt versions
    m_stream.setVersion(QDataStream::Qt_4_6);

    QString file_type;
    int version;
    m_stream >> file_type >> version;

    if (file_type != file_prefix || version != 0)
    {
        return false;
    }

    return true;
}

bool ProtobufFileReader::readNext(google::protobuf::Message &message)
{
    if (m_stream.atEnd())
    {
        return false;
    }
    QByteArray data;
    m_stream >> data;

    if (!message.ParseFromArray(data.data(), data.size()))
    {
        return false;
    }
    return true;
}
