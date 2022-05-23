/***************************************************************************
 *   Copyright 2020 Michael Eischer, Andreas Wendler                       *
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

#ifndef TEXTURECACHE_H
#define TEXTURECACHE_H

#include <QtCore/QHash>
#include <QtCore/QMultiMap>
#include <QtOpenGL/QGLContext>

class TextureCache
{
   public:
    explicit TextureCache(QGLContext *context);
    ~TextureCache();
    TextureCache(const TextureCache &) = delete;
    TextureCache &operator=(const TextureCache &) = delete;

    bool find(const QString &key, GLuint *textureId, QPixmap *pixmap);
    GLuint insert(const QString &key, const QPixmap &pixmap);
    void remove(const QString &key);

   private:
    int calcSize(const QPixmap &pixmap);

    struct TextureEntry
    {
        GLuint textureId;
        qint64 lastUsed;
        QPixmap pixmap;
    };

    QGLContext *m_context;
    QHash<QString, TextureEntry *> m_entries;
    QMultiMap<qint64, QString> m_lastUsed;
    int m_cacheSize;
    const int m_sizeLimit;
};

#endif  // TEXTURECACHE_H
