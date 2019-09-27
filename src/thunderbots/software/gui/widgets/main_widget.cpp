#include "software/gui/widgets/main_widget.h"

// TODO: comment. this is the autogenerated file from the ui file
#include "software/gui/ui/ui_main_widget.h"

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtCore/QList>
#include <QtWidgets/QSpacerItem>
#include <iostream>
#include <random>

#include "software/ai/hl/stp/play/play_factory.h"
#include <g3log/g3log.hpp>
#include "software/util/parameter/dynamic_parameters.h"

MainWidget::MainWidget(QWidget* parent)
    : QWidget(parent),
      main_widget(new Ui::AutoGeneratedMainWidget()),
      first_draw_call(true)
{
    // Handles all the setup of the generated UI components and adds the components
    // to this widget
    main_widget->setupUi(this);
    // StrongFocus means that the MainWidget will more aggressively capture focus when
    // clicked. Specifically, we do this so that when the user clicks outside of the
    // QLineEdits used for Parameters, the QLineEdit will lose focus.
    // https://www.qtcentre.org/threads/41128-Need-to-implement-in-place-line-edit-unable-to-get-lose-focus-of-QLineEdit
    setFocusPolicy(Qt::StrongFocus);

    // Replace the graphics view with a custom zoomable graphics view.
    // We do this in code rather than from the *.ui file (promoting a custom widget) because the
    // autogenerated code doesn't use absolute include paths and therefore doesn't work with bazel
    auto graphics_view_parent = main_widget->ai_visualization_graphics_view->parentWidget();
    delete main_widget->ai_visualization_graphics_view;
    main_widget->ai_visualization_graphics_view = new ZoomableQGraphicsView(graphics_view_parent);

    scene    = new QGraphicsScene(main_widget->ai_visualization_graphics_view);
    glWidget = new QOpenGLWidget(this);

    // This is a trick to force the initial width of the ai control tabs to be small,
    // and the initial width of the ai view to be large. This sets the sizes of the
    // widgets in the splitter to be unrealistically small (1 pixel) so that the
    // size policies defined for the widgets will take over and grow the widgets to
    // their minimum size, and then distribute the rest of the space according to the
    // policies.
    // See https://doc.qt.io/archives/qt-4.8/qsplitter.html#setSizes
    int number_of_widgets_in_splitter =
        main_widget->ai_control_and_view_splitter->count();
    auto widget_sizes_vector  = std::vector<int>(number_of_widgets_in_splitter, 1);
    auto widget_sizes_qvector = QVector<int>::fromStdVector(widget_sizes_vector);
    auto widget_sizes_list    = QList<int>::fromVector(widget_sizes_qvector);
    main_widget->ai_control_and_view_splitter->setSizes(widget_sizes_list);
    setupSceneView(main_widget->ai_visualization_graphics_view, scene, glWidget);

    setupStatusTable(main_widget->robot_status_table_widget);
    setupAIControls();
    setupParametersTab();
}

MainWidget::~MainWidget()
{
    delete main_widget;
    delete scene;
    delete glWidget;
}

void MainWidget::updatePlayInfo(PlayInfo play_info) {
    QString play_type_string = QString("Play Type: %1\n").arg(QString::fromStdString(play_info.play_type));
    QString play_name_string = QString("Play Name: %1\n").arg(QString::fromStdString(play_info.play_name));
    QString tactics_string = QString("Tactics:\n");
    for(const auto& tactic_string : play_info.robot_tactic_assignment) {
        tactics_string.append(QString::fromStdString(tactic_string)).append("\n");
    }

    QString play_info_string = QString("%1\n%2\n%3").arg(play_type_string, play_name_string, tactics_string);

    main_widget->play_and_tactic_info_text_edit->setText(play_info_string);
}

void MainWidget::updateRobotStatus(std::vector<std::pair<std::string, Duration>> robot_status_messages) {
    setRobotStatus(main_widget->robot_status_table_widget, robot_status_messages);
}

void MainWidget::setupAIControls()
{
    setupAIStartAndStopButtons();
    setupTeamColourComboBox();
    setupDefendingSideComboBox();
    setupGameStateOverrideComboBox();
    setupPlayOverrideComboBox();
}

void MainWidget::setupAIStartAndStopButtons()
{
    auto start_ai_func = []() { Util::DynamicParameters::AI::run_ai.setValue(true); };
    connect(main_widget->start_ai_button, &QPushButton::clicked, start_ai_func);
    auto stop_ai_func = []() { Util::DynamicParameters::AI::run_ai.setValue(false); };
    connect(main_widget->stop_ai_button, &QPushButton::clicked, stop_ai_func);
}

void MainWidget::setupTeamColourComboBox()
{
    main_widget->team_colour_combo_box->insertItem(0, "Yellow");
    main_widget->team_colour_combo_box->insertItem(1, "Blue");
    main_widget->team_colour_combo_box->insertItem(2, "Use Refbox");
    auto on_team_colour_changed = [](const QString& text) {
        if (text == "Yellow")
        {
            Util::DynamicParameters::AI::refbox::override_refbox_friendly_team_color
                .setValue(true);
            Util::DynamicParameters::AI::refbox::friendly_color_yellow.setValue(true);
        }
        else if (text == "Blue")
        {
            Util::DynamicParameters::AI::refbox::override_refbox_friendly_team_color
                .setValue(true);
            Util::DynamicParameters::AI::refbox::friendly_color_yellow.setValue(false);
        }
        else if (text == "Use Refbox")
        {
            Util::DynamicParameters::AI::refbox::override_refbox_friendly_team_color
                .setValue(false);
        }
        else
        {
            LOG(FATAL) << "Tried to set the team colour with an invalid value: '"
                       << text.toStdString() << "'" << std::endl;
        }
    };
    connect(main_widget->team_colour_combo_box, &QComboBox::currentTextChanged,
            on_team_colour_changed);
}

void MainWidget::setupGameStateOverrideComboBox()
{
    // TODO: Set this up using factory values like the play override once a factory of
    // these values is available
    main_widget->gamestate_override_combo_box->insertItem(0, "None");
    main_widget->gamestate_override_combo_box->insertItem(1, "Play");
    main_widget->gamestate_override_combo_box->insertItem(2, "Halt");
    main_widget->gamestate_override_combo_box->insertItem(3, "Stop");

    auto on_gamestate_changed = [](const QString& text) {
        if (text == "Use Refbox")
        {
            Util::DynamicParameters::AI::refbox::override_refbox_play.setValue(false);
        }
        else
        {
            Util::DynamicParameters::AI::refbox::override_refbox_play.setValue(true);
            Util::DynamicParameters::AI::refbox::current_refbox_play.setValue(
                text.toStdString());
        }
    };
    connect(main_widget->gamestate_override_combo_box, &QComboBox::currentTextChanged,
            on_gamestate_changed);
}

void MainWidget::setupPlayOverrideComboBox()
{
    auto play_names = PlayFactory::getRegisteredPlayNames();
    // Sort the entries in alphabetical order from a-z
    std::sort(play_names.begin(), play_names.end());

    // Create a new list with all the play names converted to QStrings
    QList<QString> qt_play_names;
    std::transform(play_names.begin(), play_names.end(),
                   std::back_inserter(qt_play_names),
                   [](std::string name) { return QString::fromStdString(name); });

    main_widget->play_override_combo_box->insertItem(0, "Use AI Selection");
    main_widget->play_override_combo_box->insertItems(1, qt_play_names);

    auto on_play_changed = [](const QString& text) {
        if (text == "Use AI Selection")
        {
            Util::DynamicParameters::AI::override_ai_play.setValue(false);
        }
        else
        {
            Util::DynamicParameters::AI::override_ai_play.setValue(false);
            Util::DynamicParameters::AI::current_ai_play.setValue(text.toStdString());
        }
    };
    connect(main_widget->play_override_combo_box, &QComboBox::currentTextChanged,
            on_play_changed);
}

void MainWidget::setupDefendingSideComboBox()
{
    main_widget->defending_side_combo_box->insertItem(0, "Use Refbox");
    main_widget->defending_side_combo_box->insertItem(1, "East");
    main_widget->defending_side_combo_box->insertItem(2, "West");

    auto on_defending_side_changed = [](const QString& text) {
        if (text == "Use Refbox")
        {
            Util::DynamicParameters::AI::refbox::override_refbox_defending_side.setValue(
                false);
        }
        else if (text == "East")
        {
            Util::DynamicParameters::AI::refbox::override_refbox_defending_side.setValue(
                true);
            // TODO: Confirm how East and West map to positive and negative sides
            Util::DynamicParameters::AI::refbox::defending_positive_side.setValue(false);
        }
        else if (text == "West")
        {
            Util::DynamicParameters::AI::override_ai_play.setValue(false);
            // TODO: Confirm how East and West map to positive and negative sides
            Util::DynamicParameters::AI::refbox::defending_positive_side.setValue(true);
        }
        else
        {
            LOG(FATAL) << "Tried to set the defending side with an invalid value: '"
                       << text.toStdString() << "'" << std::endl;
        }
    };
    connect(main_widget->defending_side_combo_box, &QComboBox::currentTextChanged,
            on_defending_side_changed);
}

void MainWidget::draw(WorldDrawFunction world_draw_function, AIDrawFunction ai_draw_function) {
    scene->clear();
    world_draw_function.execute(scene);
    ai_draw_function.execute(scene);
    if (first_draw_call)
    {
        main_widget->ai_visualization_graphics_view->fitInView(scene->sceneRect(),
                                                               Qt::KeepAspectRatio);
        first_draw_call = false;
    }
}

void MainWidget::updateRobotStatusMessages()
{
//    std::random_device dev;
//    std::mt19937 rng(dev());
//    std::uniform_int_distribution<std::mt19937::result_type> dist(0, 10);
//
//    int num_entries = dist(rng);
//
//    std::vector<std::string> status_msgs;
//    for (int i = 0; i < num_entries; i++)
//    {
//        int age         = dist(rng);
//        std::string msg = "Robot " + std::to_string(age) +
//                          "  --  some error oh no. Number " + std::to_string(age);
//        status_msgs.emplace_back(msg);
//    }

//    setRobotStatus(main_widget->robot_status_table_widget, status_msgs);
}

void MainWidget::setupSceneView(QGraphicsView* view, QGraphicsScene* scene,
                                QOpenGLWidget* gl_widget)
{
    view->setScene(scene);
    view->setDragMode(QGraphicsView::ScrollHandDrag);
    view->setBackgroundBrush(QBrush(Qt::darkGreen, Qt::SolidPattern));

    // Performance optimizations
    // https://stackoverflow.com/questions/43826317/how-to-optimize-qgraphicsviews-performance
    view->setInteractive(false);
    view->setOptimizationFlag(QGraphicsView::DontAdjustForAntialiasing);
    view->setCacheMode(QGraphicsView::CacheBackground);
    view->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
    // Using an OpenGL widget with the view should help make use of the graphics card
    // rather than doing CPU drawing, which should take some load off the CPU and make
    // things faster
    view->setViewport(gl_widget);

    // Invert the y-coordinates of the view.
    // We do this because Qt's default coordinate system for drawing is:
    // * positive x = "right"
    // * positive y = "down"
    // Our coordinate system defines positive y as being "up", so we invert the coordinate
    // system so all our draw calls can follow our coordinate convention. This also fixes
    // the orientation convention, so "positive" rotation is counterclockwise in the view
    // (rotating from +x, to +y, to -x, to -y)
    QTransform view_transform(1, 0, 0, -1, 0, 0);
    view->setTransform(view_transform);

    view->update();
}

void MainWidget::setupStatusTable(QTableWidget* table)
{
    QList<QString> horizontal_header_labels({"Age (Seconds)", "Message"});
    // The column count must be set before the labels are set
    table->setColumnCount(horizontal_header_labels.size());
    table->setHorizontalHeaderLabels(horizontal_header_labels);
    table->horizontalHeader()->setVisible(true);
    table->horizontalHeader()->setStretchLastSection(true);

    table->verticalHeader()->setVisible(false);
    table->setShowGrid(true);
    table->setVisible(true);

    table->update();
}

void MainWidget::setRobotStatus(QTableWidget* table,
                                std::vector<std::pair<std::string, Duration>> robot_status_messages)
{
    // Resize the number of rows to only have as many rows as we have messages. This will
    // automatically delete any extra rows / messages for us, and then we overwrite the
    // existing rows with new messages
    table->setRowCount(robot_status_messages.size());
    int row = 0;
    for (const auto& status_message : robot_status_messages)
    {
        auto [message, age_duration] = status_message;
        QString age = QString::number(std::floor(age_duration.getSeconds()));
        table->setItem(row, 0, new QTableWidgetItem(age));
        table->setItem(row, 1, new QTableWidgetItem(QString::fromStdString(message)));
        row++;
    }
}

QWidget* MainWidget::createBooleanParameter(Parameter<bool>* parameter)
{
    QWidget* widget     = new QWidget();
    QHBoxLayout* layout = new QHBoxLayout(widget);

    QLabel* label = new QLabel(widget);
    label->setText(QString::fromStdString(parameter->name()));
    QCheckBox* checkbox = new QCheckBox(widget);
    checkbox->setChecked(parameter->value());

    layout->addWidget(label);
    layout->addWidget(checkbox);

    auto on_checkbox_value_changed = [parameter, checkbox]() {
        LOG(INFO) << "Value for boolean param " << parameter->name() << " changed to "
                  << checkbox->isChecked() << std::endl;
        parameter->setValue(checkbox->isChecked());
    };
    connect(checkbox, &QCheckBox::stateChanged, on_checkbox_value_changed);

    auto on_parameter_value_changed = [checkbox](bool new_value) {
        checkbox->setChecked(new_value);
    };
    parameter->registerCallbackFunction(on_parameter_value_changed);

    widget->setLayout(layout);

    return widget;
}

QWidget* MainWidget::createIntegerParameter(Parameter<int>* parameter)
{
    QWidget* widget     = new QWidget();
    QHBoxLayout* layout = new QHBoxLayout(widget);

    QLabel* label = new QLabel(widget);
    label->setText(QString::fromStdString(parameter->name()));
    QSpinBox* spinbox = new QSpinBox(widget);
    // TODO: Get range from parameter
    spinbox->setRange(0, 100);
    spinbox->setValue(parameter->value());

    layout->addWidget(label);
    layout->addWidget(spinbox);

    auto on_spinbox_value_changed = [parameter, spinbox]() {
        LOG(INFO) << "Value for integer param " << parameter->name() << " changed to "
                  << spinbox->value() << std::endl;
        parameter->setValue(spinbox->value());
    };
    // QSpinBox has 2 "valueChanged" signals that each provide different info (string vs
    // int), so we need to static_cast to specify the integer version
    connect(spinbox, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
            on_spinbox_value_changed);

    auto on_parameter_value_changed = [spinbox](int new_value) {
        spinbox->setValue(new_value);
    };
    parameter->registerCallbackFunction(on_parameter_value_changed);

    widget->setLayout(layout);

    return widget;
}

QWidget* MainWidget::createDoubleParameter(Parameter<double>* parameter)
{
    QWidget* widget     = new QWidget();
    QHBoxLayout* layout = new QHBoxLayout(widget);

    QLabel* label = new QLabel(widget);
    label->setText(QString::fromStdString(parameter->name()));
    QDoubleSpinBox* spinbox = new QDoubleSpinBox(widget);
    // TODO: Get range from parameter
    spinbox->setRange(0, 100);
    spinbox->setValue(parameter->value());
    spinbox->setSingleStep(0.05);

    layout->addWidget(label);
    layout->addWidget(spinbox);

    auto on_spinbox_value_changed = [parameter, spinbox]() {
        LOG(INFO) << "Value for double param " << parameter->name() << " changed to "
                  << spinbox->value() << std::endl;
        parameter->setValue(spinbox->value());
    };
    // QDoubleSpinBox has 2 "valueChanged" signals that each provide different info
    // (string vs int), so we need to static_cast to specify the integer version
    connect(spinbox,
            static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            on_spinbox_value_changed);

    auto on_parameter_value_changed = [spinbox](int new_value) {
        spinbox->setValue(new_value);
    };
    parameter->registerCallbackFunction(on_parameter_value_changed);

    widget->setLayout(layout);

    return widget;
}

QWidget* MainWidget::createStringParameter(Parameter<std::string>* parameter)
{
    QWidget* widget     = new QWidget();
    QHBoxLayout* layout = new QHBoxLayout(widget);

    QLabel* label = new QLabel(widget);
    label->setText(QString::fromStdString(parameter->name()));
    QLineEdit* line_edit = new QLineEdit(widget);
    line_edit->setText(QString::fromStdString(parameter->value()));

    layout->addWidget(label);
    layout->addWidget(line_edit);

    auto on_line_edit_text_changed = [parameter, line_edit]() {
        LOG(INFO) << "Value for string param " << parameter->name() << " changed to "
                  << line_edit->text().toStdString() << std::endl;
        parameter->setValue(line_edit->text().toStdString());
    };
    // This event will only fire when "Enter" is pressed or the LineEdit loses focus,
    // rather than every time a character changes in the LineEdit.
    // https://doc.qt.io/archives/qt-4.8/qlineedit.html#editingFinished
    connect(line_edit, &QLineEdit::editingFinished, on_line_edit_text_changed);

    auto on_parameter_value_changed = [line_edit](std::string new_value) {
        line_edit->setText(QString::fromStdString(new_value));
    };
    parameter->registerCallbackFunction(on_parameter_value_changed);

    widget->setLayout(layout);

    return widget;
}

void MainWidget::setupParametersTab()
{
    auto tab                = main_widget->params_tab;
    auto params_scroll_area = main_widget->params_tab_scroll_area;
    QWidget* widget         = new QWidget(tab);
    QVBoxLayout* layout     = new QVBoxLayout(widget);
    params_scroll_area->setWidget(widget);
    params_scroll_area->setWidgetResizable(true);

    for (auto param : Parameter<bool>::getRegistry())
    {
        QWidget* bool_param_widget = createBooleanParameter(param);
        bool_param_widget->setParent(widget);
        layout->addWidget(bool_param_widget);
    }

    for (auto param : Parameter<int>::getRegistry())
    {
        QWidget* int_param_widget = createIntegerParameter(param);
        int_param_widget->setParent(widget);
        layout->addWidget(int_param_widget);
    }

    for (auto param : Parameter<double>::getRegistry())
    {
        QWidget* double_param_widget = createDoubleParameter(param);
        double_param_widget->setParent(widget);
        layout->addWidget(double_param_widget);
    }

    for (auto param : Parameter<std::string>::getRegistry())
    {
        QWidget* string_param_widget = createStringParameter(param);
        string_param_widget->setParent(widget);
        layout->addWidget(string_param_widget);
    }

    layout->addSpacerItem(
        new QSpacerItem(20, 40, QSizePolicy::Expanding, QSizePolicy::Expanding));
    main_widget->params_tab->show();
}
