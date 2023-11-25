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

#include "protobuffilesaver.h"

#include <QtCore/QDebug>

ProtobufFileSaver::ProtobufFileSaver(QString filename, QString filePrefix,
                                     QObject *parent)
    : QObject(parent),
      m_filename(filename),
      m_filePrefix(filePrefix),
      m_stream(&m_file),
      m_mutex(QMutex::Recursive)
{
}

void ProtobufFileSaver::open()
{
    if (m_file.isOpen())
    {
        return;
    }
    m_file.setFileName(m_filename);
    if (!m_file.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        qDebug() << "Could not open path input file for saving";
        return;
    }

    // ensure compatibility across qt versions
    m_stream.setVersion(QDataStream::Qt_4_6);

    m_stream << QString(m_filePrefix);
    m_stream << (int)0;  // file version
}

void ProtobufFileSaver::saveMessage(const google::protobuf::Message &message)
{
    QByteArray data;
    data.resize(message.ByteSize());
    if (!message.IsInitialized() || !message.SerializeToArray(data.data(), data.size()))
    {
        return;
    }

    QMutexLocker locker(&m_mutex);
    open();

    m_stream << data;
}
