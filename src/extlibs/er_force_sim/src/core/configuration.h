/****************************************************************************
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

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <google/protobuf/message.h>
#include <google/protobuf/text_format.h>

#include <QtCore/QFile>
#include <QtCore/QString>
#include <iostream>

#include "src/config/config.h"

inline bool loadConfiguration(const QString &config_file,
                              google::protobuf::Message *message, bool allow_partial)
{
    QString full_filename = QString(ERFORCE_CONFDIR) + config_file + ".txt";
    QFile file(full_filename);
    if (!file.open(QFile::ReadOnly))
    {
        std::cout << "Could not open configuration file " << full_filename.toStdString()
                  << std::endl;
        return false;
    }
    QString str = file.readAll();
    file.close();
    std::string s = qPrintable(str);

    google::protobuf::TextFormat::Parser parser;
    parser.AllowPartialMessage(allow_partial);
    parser.ParseFromString(s, message);
    return true;
}

#endif  // CONFIGURATION_H
