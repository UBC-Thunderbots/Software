/***************************************************************************
 *   Copyright 2015 Michael Eischer, Jan Kallwies, Philipp Nordhus         *
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

#include "plotter.h"

#include <QtGui/QCloseEvent>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QMenu>
#include <QtCore/QSettings>
#include <QtCore/QStringBuilder>
#include <cmath>
#include <unordered_map>

#include "google/protobuf/descriptor.h"
#include "leaffilterproxymodel.h"
#include "plot.h"
#include "extlibs/er_force_plotter/ui_plotter.h"

Plotter::Plotter()
    : QWidget(nullptr, Qt::Window),
      ui(new Ui::Plotter),
      m_startTime(0),
      m_freeze(false),
      m_playingBacklog(false)
{
    setWindowIcon(QIcon("icon:plotter.svg"));
    ui->setupUi(this);

    // proxy model for tree filtering
    m_proxy = new LeafFilterProxyModel(this);
    m_proxy->setFilterCaseSensitivity(Qt::CaseInsensitive);
    m_proxy->setSourceModel(&m_model);
    ui->tree->setUniformRowHeights(true);
    ui->tree->setModel(m_proxy);
    connect(ui->lineSearch, SIGNAL(textChanged(QString)), m_proxy,
            SLOT(setFilterFixedString(QString)));
    connect(ui->lineSearch, SIGNAL(returnPressed()), ui->lineSearch, SLOT(clear()));

    // root items in the plotter
    addRootItem(QStringLiteral("Ball"), QStringLiteral("Ball"));
    addRootItem(QStringLiteral("Yellow"), QStringLiteral("Yellow robots"));
    addRootItem(QStringLiteral("YellowStrategy"), QStringLiteral("Yellow strategy"));
    addRootItem(QStringLiteral("Blue"), QStringLiteral("Blue robots"));
    addRootItem(QStringLiteral("BlueStrategy"), QStringLiteral("Blue strategy"));
    addRootItem(QStringLiteral("RadioCommand"), QStringLiteral("Radio commands"));
    addRootItem(QStringLiteral("RadioResponse"), QStringLiteral("Radio responses"));
    addRootItem(QStringLiteral("Timing"), QStringLiteral("Timing"));

    ui->tree->expandAll();  // expands the root items, thus childs are immediatelly
                            // visible once added
    connect(&m_model, SIGNAL(itemChanged(QStandardItem *)),
            SLOT(itemChanged(QStandardItem *)));

    // connect freeze
    connect(ui->btnFreeze, SIGNAL(toggled(bool)), this, SLOT(setFreeze(bool)));

    // connect the plot widget
    m_timeLimit = ui->spinDuration->maximum();
    connect(ui->spinYMin, SIGNAL(valueChanged(double)), ui->widget,
            SLOT(setYMin(double)));
    connect(ui->spinYMax, SIGNAL(valueChanged(double)), ui->widget,
            SLOT(setYMax(double)));
    connect(ui->spinDuration, SIGNAL(valueChanged(double)), ui->widget,
            SLOT(setDuration(double)));
    connect(this, SIGNAL(addPlot(const Plot *)), ui->widget, SLOT(addPlot(const Plot *)));
    connect(this, SIGNAL(removePlot(const Plot *)), ui->widget,
            SLOT(removePlot(const Plot *)));
    // connect the time axis scroll bar
    // scrolling creates a negative time offset
    ui->timeScroll->setMaximum(0);
    ui->timeScroll->setValue(0);  // scroll to the latest values
    connect(ui->spinDuration, SIGNAL(valueChanged(double)), SLOT(updateScrollBar()));
    connect(ui->timeScroll, SIGNAL(valueChanged(int)), SLOT(updateOffset(int)));
    updateScrollBar();
    // redirect scroll events on the widget
    ui->widget->installEventFilter(this);

    QAction *actionSpacePressed = new QAction(this);
    actionSpacePressed->setShortcut(QKeySequence(Qt::Key_Space));
    connect(actionSpacePressed, &QAction::triggered, this, &Plotter::spacePressed);
    addAction(actionSpacePressed);

    // setup context menu for plot list
    m_plotMenu           = new QMenu(this);
    QAction *actionClear = new QAction(QStringLiteral("Clear selection"), m_plotMenu);
    connect(actionClear, SIGNAL(triggered()), this, SLOT(clearSelection()));
    m_plotMenu->addAction(actionClear);

    ui->tree->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(ui->tree, SIGNAL(customContextMenuRequested(QPoint)), this,
            SLOT(plotContextMenu(QPoint)));

    // restore geometry
    QSettings s;
    s.beginGroup(QStringLiteral("Plotter"));
    restoreGeometry(s.value(QStringLiteral("geometry")).toByteArray());
    ui->splitter->restoreState(s.value(QStringLiteral("splitter")).toByteArray());
    ui->tree->header()->restoreState(s.value(QStringLiteral("tree")).toByteArray());
    s.endGroup();

    loadScaling();

    // TODO-AKHIL
    // setup invalidate timer
    // m_guiTimer = new GuiTimer(1000, this);
    // connect(m_guiTimer, &GuiTimer::timeout, this, &Plotter::invalidatePlots);

    loadSelection();
}

Plotter::~Plotter()
{
    delete ui;
    qDeleteAll(m_plots);
    qDeleteAll(m_frozenPlots);
}

void Plotter::closeEvent(QCloseEvent *event)
{
    QSettings s;
    s.beginGroup(QStringLiteral("Plotter"));
    s.setValue(QStringLiteral("geometry"), saveGeometry());
    s.setValue(QStringLiteral("splitter"), ui->splitter->saveState());
    s.setValue(QStringLiteral("tree"), ui->tree->header()->saveState());
    s.setValue(QStringLiteral("visible"), QStringList(m_selection.toList()));
    s.setValue(QStringLiteral("scaling"),
               QList<QVariant>({ui->spinYMin->value(), ui->spinYMax->value(),
                                ui->spinDuration->value()}));
    s.endGroup();

    // just accepting and closing leads to segfaults when re-opening the widget
    this->hide();
    event->ignore();
}

void Plotter::setScaling(float min, float max, float timespan)
{
    ui->spinYMin->setValue(min);
    ui->spinYMax->setValue(max);
    ui->spinDuration->setValue(timespan);
}

void Plotter::addRootItem(const QString &name, const QString &displayName)
{
    QStandardItem *item = new QStandardItem(displayName);
    m_model.appendRow(item);
    m_items[name] = item;
}

void Plotter::loadScaling()
{
    QSettings s;
    s.beginGroup(QStringLiteral("Plotter"));
    QList<QVariant> scaling = s.value(QStringLiteral("scaling")).toList();
    if (scaling.size() == 3)
    {
        float ymin     = scaling[0].toFloat();
        float ymax     = scaling[1].toFloat();
        float duration = scaling[2].toFloat();
        setScaling(ymin, ymax, duration);
    }
    s.endGroup();
}

void Plotter::loadSelection()
{
    QSettings s;
    s.beginGroup(QStringLiteral("Plotter"));

    m_selection.clear();
    QString vis = QStringLiteral("visible");
    foreach (const QString &s, s.value(vis).toStringList())
    {
        m_selection.insert(s);
    }

    s.endGroup();
}

void Plotter::plotContextMenu(const QPoint &pos)
{
    m_plotMenu->popup(ui->tree->mapToGlobal(pos));
}

void Plotter::clearSelection()
{
    foreach (QStandardItem *item, m_items)
    {
        // only modify checked plots
        if (item->isCheckable() && item->checkState() == Qt::Checked)
        {
            item->setCheckState(Qt::Unchecked);
        }
    }
    // clear selection afterwards, as the itemChanged won't fire otherwise
    m_selection.clear();
}

void Plotter::updateScrollBar()
{
    double duration = ui->spinDuration->value();

    // max - min + pageStep = full time scale
    ui->timeScroll->setMinimum(-(m_timeLimit - duration) * 100);
    ui->timeScroll->setSingleStep(duration * 10);  // scroll a tenth of the screen width
    ui->timeScroll->setPageStep(duration * 100);
}

void Plotter::updateOffset(int pos)
{
    // divide by 100 to allow fine scrolling
    ui->widget->setOffset(pos / 100.);
}

bool Plotter::eventFilter(QObject *obj, QEvent *event)
{
    if (event->type() == QEvent::Wheel)
    {
        // forward scroll events to scrollbar
        ui->timeScroll->event(event);
        return true;
    }
    else
    {
        // standard event processing
        return QObject::eventFilter(obj, event);
    }
}

void Plotter::setFreeze(bool freeze)
{
    if (!freeze && m_freeze)
    {
        // merge plots on unfreezing
        foreach (QStandardItem *item, m_items)
        {
            Plot *freezePlot = m_frozenPlots.value(item, nullptr);
            if (freezePlot == nullptr)
            {
                continue;
            }
            // remove freeze plot entry
            m_frozenPlots.remove(item);

            Plot *plot = m_plots.value(item, nullptr);
            if (plot != nullptr)
            {  // merge freeze plot if there's already a plot
                plot->mergeFrom(freezePlot);
                delete freezePlot;
            }
            else
            {  // otherwise reuse it as plot
                m_plots[item] = freezePlot;
            }
        }
    }
    m_freeze = freeze;
    ui->btnFreeze->setChecked(freeze);  // update button
}

void Plotter::handleUiResponse(const amun::UiResponse &response, qint64 time)
{
    // TODO-AKHIL
    m_playingBacklog = true;
    for (const amun::Status &st : response.logger_status())
    {
        // Status s = Status::createArena();
        // s->CopyFrom(st);
        // QCoreApplication::processEvents();
    }
    for (int i = 0; i < m_backlog.size(); i++)
    {
        // handleStatus(m_backlog[i], true);
        // QCoreApplication::processEvents();
    }
    m_backlog.clear();
    m_playingBacklog = false;
}

void Plotter::showPlotter()
{
    // all incoming data has to wait until the backlog-status are consumed
    m_playingBacklog = true;
    show();
}

QStandardItem *Plotter::getItem(const QString &name)
{
    // item already exists
    if (m_items.contains(name))
    {
        return m_items[name];
    }

    int split = name.lastIndexOf(QChar('.'));
    if (split == -1)
    {  // silently handle the case that the root item is missing
        addRootItem(name, name);
        return m_items[name];
    }

    // create new item and add it to the model
    const QString parentName = name.mid(0, split);
    const QString childName  = name.mid(split + 1);
    QStandardItem *parent    = getItem(parentName);
    QStandardItem *child     = new QStandardItem(childName);
    child->setData(name, Plotter::FullNameRole);
    m_items[name] = child;
    parent->appendRow(child);
    return child;
}

void Plotter::invalidatePlots()
{
    if (!isVisible())
    {  // values aren't update while hidden
        return;
    }

    const float time = (m_time - m_startTime) * 1E-9f;

    foreach (QStandardItem *item, m_items)
    {
        // check the role that is currently updated
        QHash<QStandardItem *, Plot *> &plots = (m_freeze) ? m_frozenPlots : m_plots;
        Plot *plot                            = plots.value(item, nullptr);
        if (plot == nullptr)
        {
            continue;
        }
        if (plot->time() + 5 < time)
        {
            // mark old plots
            item->setForeground(Qt::gray);
        }
    }
}

enum class SpecialFieldNames : int
{
    none         = 0,
    v_f          = 1,
    v_s          = 2,
    v_x          = 3,
    v_y          = 4,
    v_d_x        = 5,
    v_d_y        = 6,
    v_ctrl_out_f = 7,
    v_ctrl_out_s = 8,
    area         = 9,
    max          = 10
};

static const std::unordered_map<std::string, SpecialFieldNames> fieldNameMap = {
    std::make_pair("v_f", SpecialFieldNames::v_f),
    std::make_pair("v_s", SpecialFieldNames::v_s),
    std::make_pair("v_x", SpecialFieldNames::v_x),
    std::make_pair("v_y", SpecialFieldNames::v_y),
    std::make_pair("v_desired_x", SpecialFieldNames::v_d_x),
    std::make_pair("v_desired_y", SpecialFieldNames::v_d_y),
    std::make_pair("v_ctrl_out_f", SpecialFieldNames::v_ctrl_out_f),
    std::make_pair("v_ctrl_out_s", SpecialFieldNames::v_ctrl_out_s),
    std::make_pair("area", SpecialFieldNames::area),
};

void Plotter::parseMessage(const google::protobuf::Message &message,
                           const QString &parent, float time)
{
    const google::protobuf::Descriptor *desc = message.GetDescriptor();
    const google::protobuf::Reflection *refl = message.GetReflection();

    float specialFields[static_cast<int>(SpecialFieldNames::max)];
    for (int i = 0; i < static_cast<int>(SpecialFieldNames::max); ++i)
    {
        specialFields[i] = NAN;
    }

    const int extraFields = 4;
    if (!m_itemLookup.contains(parent))
    {
        m_itemLookup[parent] =
            QVector<QStandardItem *>(desc->field_count() + extraFields, nullptr);
    }
    // just a performance optimization
    QVector<QStandardItem *> &childLookup = m_itemLookup[parent];

    for (int i = 0; i < desc->field_count(); i++)
    {
        const google::protobuf::FieldDescriptor *field = desc->field(i);

        // check type and that the field exists
        if (field->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_FLOAT &&
            refl->HasField(message, field))
        {
            const std::string &name = field->name();
            const float value       = refl->GetFloat(message, field);
            if (fieldNameMap.count(name) > 0)
            {
                SpecialFieldNames fn                = fieldNameMap.at(name);
                specialFields[static_cast<int>(fn)] = value;
            }
            addPoint(name, parent, time, value, childLookup, i);
        }
        else if (field->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_BOOL &&
                 refl->HasField(message, field))
        {
            const std::string &name = field->name();
            const float value       = refl->GetBool(message, field) ? 1 : 0;
            addPoint(name, parent, time, value, childLookup, i);
        }
    }

    // precompute strings
    static const std::string staticVLocal("v_local");
    static const std::string staticVDesired("v_desired");
    static const std::string staticVCtrlOut("v_ctrl_out");
    static const std::string staticVGlobal("v_global");

    // add length of speed vectors
    tryAddLength(staticVLocal, parent, time,
                 specialFields[static_cast<int>(SpecialFieldNames::v_f)],
                 specialFields[static_cast<int>(SpecialFieldNames::v_s)], childLookup,
                 desc->field_count() + 0);
    tryAddLength(staticVDesired, parent, time,
                 specialFields[static_cast<int>(SpecialFieldNames::v_d_x)],
                 specialFields[static_cast<int>(SpecialFieldNames::v_d_y)], childLookup,
                 desc->field_count() + 1);
    tryAddLength(staticVCtrlOut, parent, time,
                 specialFields[static_cast<int>(SpecialFieldNames::v_ctrl_out_f)],
                 specialFields[static_cast<int>(SpecialFieldNames::v_ctrl_out_f)],
                 childLookup, desc->field_count() + 2);
    tryAddLength(staticVGlobal, parent, time,
                 specialFields[static_cast<int>(SpecialFieldNames::v_x)],
                 specialFields[static_cast<int>(SpecialFieldNames::v_y)], childLookup,
                 desc->field_count() + 3);
}

void Plotter::tryAddLength(const std::string &name, const QString &parent, float time,
                           float value1, float value2,
                           QVector<QStandardItem *> &childLookup, int descriptorIndex)
{
    // if both values are set
    if (!std::isnan(value1) && !std::isnan(value2))
    {
        const float value = std::sqrt(value1 * value1 + value2 * value2);
        addPoint(name, parent, time, value, childLookup, descriptorIndex);
    }
}

void Plotter::addPoint(const std::string &name, const QString &parent, float time,
                       float value, QVector<QStandardItem *> &childLookup,
                       int descriptorIndex)
{
    QStandardItem *item;
    if (childLookup.isEmpty() || childLookup[descriptorIndex] == nullptr)
    {
        // full name for item retrieval
        const QString fullName =
            parent % QStringLiteral(".") % QString::fromStdString(name);
        item = getItem(fullName);
        if (!childLookup.isEmpty())
        {
            childLookup[descriptorIndex] = item;
        }
    }
    else
    {
        item = childLookup[descriptorIndex];
    }

    // save data into a hidden plot while freezed
    QHash<QStandardItem *, Plot *> &plots = (m_freeze) ? m_frozenPlots : m_plots;
    Plot *plot                            = plots.value(item, nullptr);

    if (plot == nullptr)
    {  // create new plot
        const QString fullName =
            parent % QStringLiteral(".") % QString::fromStdString(name);
        plot = new Plot(fullName, this);
        item->setCheckable(true);
        if (m_selection.contains(fullName))
        {
            emit addPlot(plot);  // manually add plot as itemChanged won't add it
            item->setCheckState(Qt::Checked);
        }
        else
        {
            item->setCheckState(Qt::Unchecked);
        }
        // set plot information after the check state
        // itemChanged only checks items in m_plots
        // thus no enable / disable flickering will occur
        plots[item] = plot;
    }
    // only clear foreground if it's set, causes a serious performance regression
    // if it's always done
    if (item->data(Qt::ForegroundRole).isValid())
    {
        item->setData(QVariant(), Qt::ForegroundRole);  // clear foreground color
    }
    plot->addPoint(time, value);
}

void Plotter::clearData()
{
    // fix loss of precision when loading multiple log files without restarting the
    // plotter
    m_startTime = 0;
    m_guiTimer->requestTriggering();
    // delete everything
    foreach (QStandardItem *item, m_items)
    {
        // just drop the freeze plot
        if (m_frozenPlots.contains(item))
        {
            delete m_frozenPlots[item];
            m_frozenPlots.remove(item);
        }
        if (m_plots.contains(item))
        {
            m_plots[item]->clearData();
        }
    }
    // force unfreeze as no more data is available
    setFreeze(false);
}

void Plotter::itemChanged(QStandardItem *item)
{
    // always use m_plots as that's what governs which plot to display
    if (m_plots.contains(item))
    {
        Plot *plot         = m_plots[item];
        const QString name = item->data(Plotter::FullNameRole).toString();
        if (item->checkState() == Qt::Checked)
        {
            // only add plot if it isn't in our selection yet
            if (!m_selection.contains(name))
            {
                emit addPlot(plot);
                m_selection.insert(name);
            }
        }
        else
        {
            // same for remove
            if (m_selection.remove(name))
            {
                emit removePlot(plot);
            }
        }
    }
}
