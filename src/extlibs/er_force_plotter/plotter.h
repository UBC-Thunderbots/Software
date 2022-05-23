/***************************************************************************
 *   Copyright 2015 Michael Eischer, Philipp Nordhus                       *
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

#ifndef PLOTTER_H
#define PLOTTER_H

#include <QtCore/QList>
#include <QtCore/QSet>
#include <QtGui/QStandardItemModel>
#include <QtWidgets/QWidget>

#include "extlibs/er_force_sim/src/protobuf/world.pb.h"

class LeafFilterProxyModel;
class Plot;
class GuiTimer;
class QMenu;
namespace Ui
{
    class Plotter;
}

class Plotter : public QWidget
{
    Q_OBJECT

   public:
    explicit Plotter();
    ~Plotter() override;
    Plotter(const Plotter &) = delete;
    Plotter &operator=(const Plotter &) = delete;

    void showPlotter();

   public slots:
    void setScaling(float min, float max, float timespan);
    void clearData();

   signals:
    void addPlot(const Plot *plot);
    void removePlot(const Plot *plot);
    void spacePressed();

   protected:
    void closeEvent(QCloseEvent *event) override;

   private slots:
    void updateScrollBar();
    void updateOffset(int pos);

    void setFreeze(bool freeze);
    void plotContextMenu(const QPoint &pos);
    void clearSelection();
    void itemChanged(QStandardItem *item);
    void invalidatePlots();

   private:
    bool eventFilter(QObject *obj, QEvent *event) override;

    void loadScaling();
    void loadSelection();
    QStandardItem *getItem(const QString &name);
    void addRootItem(const QString &name, const QString &displayName);
    void parseMessage(const google::protobuf::Message &message, const QString &parent,
                      float time);
    void addPoint(const std::string &name, const QString &parent, float time, float value,
                  QVector<QStandardItem *> &childLookup, int descriptorIndex);
    void tryAddLength(const std::string &name, const QString &parent, float time,
                      float value1, float value2, QVector<QStandardItem *> &childLookup,
                      int descriptorIndex);

   private:
    enum ItemRole
    {
        FullNameRole = Qt::UserRole + 2
    };

    Ui::Plotter *ui;
    qint64 m_startTime;
    qint64 m_time;
    double m_timeLimit;
    bool m_freeze;
    GuiTimer *m_guiTimer;
    QHash<QString, QStandardItem *> m_items;
    QHash<QString, QVector<QStandardItem *>> m_itemLookup;
    QHash<QStandardItem *, Plot *> m_plots;
    QHash<QStandardItem *, Plot *> m_frozenPlots;
    QSet<QString> m_selection;
    QStandardItemModel m_model;
    LeafFilterProxyModel *m_proxy;
    QMenu *m_plotMenu;
    bool m_playingBacklog;
};

#endif  // PLOTTER_H
