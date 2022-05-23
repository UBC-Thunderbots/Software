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

#include "extlibs/er_force_plotter/texturecache.h"

TextureCache::TextureCache(QGLContext *context)
    : m_context(context), m_cacheSize(0), m_sizeLimit(10 * 1000 * 100)
{
}

TextureCache::~TextureCache()
{
    for (TextureEntry *entry : m_entries)
    {
        m_context->deleteTexture(entry->textureId);
    }
    qDeleteAll(m_entries);
}

bool TextureCache::find(const QString &key, GLuint *textureId, QPixmap *pixmap)
{
    if (m_entries.contains(key))
    {
        TextureEntry *entry = m_entries[key];
        *textureId          = entry->textureId;
        *pixmap             = entry->pixmap;
        // update last used
        m_lastUsed.remove(entry->lastUsed, key);
        entry->lastUsed = std::time(0);
        m_lastUsed.insertMulti(entry->lastUsed, key);
        return true;
    }
    return false;
}

GLuint TextureCache::insert(const QString &key, const QPixmap &pixmap)
{
    remove(key);

    m_cacheSize += calcSize(pixmap);
    while (m_cacheSize > m_sizeLimit && m_lastUsed.size() > 0)
    {
        // warning: delKey musn't get passed to remove as reference as it will be
        // deleted!!!
        QString delKey = m_lastUsed.first();
        remove(delKey);
    }

    TextureEntry *entry = new TextureEntry;
    entry->pixmap       = pixmap;
    entry->textureId    = m_context->bindTexture(entry->pixmap);
    entry->lastUsed     = std::time(0);
    m_entries[key]      = entry;
    m_lastUsed.insertMulti(entry->lastUsed, key);

    return entry->textureId;
}

int TextureCache::calcSize(const QPixmap &pixmap)
{
    return pixmap.width() * pixmap.height() * (pixmap.depth() / 8);
}

void TextureCache::remove(const QString &key)
{
    if (!m_entries.contains(key))
    {
        return;
    }

    TextureEntry *entry = m_entries[key];
    m_context->deleteTexture(entry->textureId);
    m_cacheSize -= calcSize(entry->pixmap);
    m_lastUsed.remove(entry->lastUsed, key);
    m_entries.remove(key);

    delete entry;
}
