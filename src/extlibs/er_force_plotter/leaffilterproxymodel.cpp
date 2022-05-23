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

#include "leaffilterproxymodel.h"

LeafFilterProxyModel::LeafFilterProxyModel(QObject *parent)
    : QSortFilterProxyModel(parent)
{
}

bool LeafFilterProxyModel::filterAcceptsRow(int source_row,
                                            const QModelIndex &source_parent) const
{
    const QModelIndex &currentItem = sourceModel()->index(source_row, 0, source_parent);

    // show if current item or any of its parents is matched
    QModelIndex parent = currentItem;
    while (parent.isValid())
    {
        if (QSortFilterProxyModel::filterAcceptsRow(parent.row(), parent.parent()))
        {
            return true;
        }
        parent = parent.parent();
    }

    // keep visible if a child is matched
    return hasAcceptedChildren(currentItem);
}

bool LeafFilterProxyModel::hasAcceptedChildren(const QModelIndex &currentItem) const
{
    if (!currentItem.isValid())
    {
        return false;
    }

    // check children
    for (int i = 0; i < currentItem.model()->rowCount(currentItem); ++i)
    {
        if (QSortFilterProxyModel::filterAcceptsRow(i, currentItem))
        {
            return true;
        }

        if (hasAcceptedChildren(sourceModel()->index(i, 0, currentItem)))
        {
            return true;
        }
    }

    return false;
}
