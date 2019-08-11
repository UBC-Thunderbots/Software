#include "software/gui/main_widget.h"

// Generated in cmake-build-*/ai_gui_autogen/include/
// Sometimes this path gets nested (eg. ai_gui_autogen/include/ai_gui_autoen/...)
// which is why we just include the file directly from the system include dirs
#include <ui_main_widget.h>

#include <iostream>
#include "software/ai/hl/stp/play/play_factory.h"

#include "gui/drawing/world.h"
#include "test/test_util/test_util.h"
#include <random>

MainWidget::MainWidget(QWidget *parent)
    : QWidget(parent), main_widget(new Ui::MainWidget()), first_draw_call(true)
{
    // Handles all the setup of the generated UI components and adds the components
    // to this widget
    main_widget->setupUi(this);

    scene = new QGraphicsScene(main_widget->ai_visualization_graphics_view);
    glWidget = new QOpenGLWidget(this);
    setupSceneView(main_widget->ai_visualization_graphics_view, scene, glWidget);

    setupStatusTable(main_widget->robot_status_table_widget);
    setupAIControls();
}

void MainWidget::setupAIControls() {
    setupAIStartAndStopButtons();
    setupTeamColourComboBox();
    setupDefendingSideComboBox();
    setupGameStateOverrideComboBox();
    setupPlayOverrideComboBox();
}

void MainWidget::setupAIStartAndStopButtons() {
    connect(main_widget->start_ai_button, &QPushButton::clicked, []() {std::cout << "start ai" << std::endl;});
    connect(main_widget->stop_ai_button, &QPushButton::clicked, []() {std::cout << "stop ai" << std::endl;});
}

void MainWidget::setupTeamColourComboBox() {
    main_widget->team_colour_combo_box->insertItem(0, "Yellow");
    main_widget->team_colour_combo_box->insertItem(1, "Blue");
    connect(main_widget->team_colour_combo_box, &QComboBox::currentTextChanged, [](const QString& text) {std::cout << "team colour changed " << text.toStdString() << std::endl;});
}

void MainWidget::setupGameStateOverrideComboBox() {
    // TODO: Set this up using factory values like the play override once a factory of these values is available
    main_widget->gamestate_override_combo_box->insertItem(0, "None");
    main_widget->gamestate_override_combo_box->insertItem(1, "Play");
    main_widget->gamestate_override_combo_box->insertItem(2, "Halt");
    main_widget->gamestate_override_combo_box->insertItem(3, "Stop");
    connect(main_widget->gamestate_override_combo_box, &QComboBox::currentTextChanged, [](const QString& text) {std::cout << "gamestate override changed " << text.toStdString() << std::endl;});
}

void MainWidget::setupPlayOverrideComboBox() {
    auto play_names = PlayFactory::getRegisteredPlayNames();
    // Sort the entries in alphabetical order from a-z
    std::sort(play_names.begin(), play_names.end());

    // Create a new list with all the play names converted to QStrings
    QList<QString> qt_play_names;
    std::transform(play_names.begin(), play_names.end(), std::back_inserter(qt_play_names), [](std::string name) {return QString::fromStdString(name);});

    main_widget->play_override_combo_box->insertItem(0, "None");
    main_widget->play_override_combo_box->insertItems(1, qt_play_names);

    connect(main_widget->play_override_combo_box, &QComboBox::currentTextChanged, [](const QString& text) {std::cout << "play override changed " << text.toStdString() << std::endl;});
}

void MainWidget::setupDefendingSideComboBox() {
    main_widget->defending_side_combo_box->insertItem(0, "East");
    main_widget->defending_side_combo_box->insertItem(1, "West");
    connect(main_widget->defending_side_combo_box, &QComboBox::currentTextChanged, [](const QString& text) {std::cout << "defending side changed " << text.toStdString() << std::endl;});
}

// TODO: Stop using pointers wherever possible, so I don't have to manually destruct stuff
MainWidget::~MainWidget()
{
    delete main_widget;
    delete scene;
    delete glWidget;
}

void MainWidget::drawAI()
{
    scene->clear();

    World world = Test::TestUtil::createBlankTestingWorld();
    world =
        Test::TestUtil::setBallPosition(world, Point(-1, 1), Timestamp::fromSeconds(0));
    world = Test::TestUtil::setBallVelocity(world, Vector(1, -1.5),
                                            Timestamp::fromSeconds(0));
    world = Test::TestUtil::setFriendlyRobotPositions(
        world, {Point(-4, 1), Point(0, -3), Point(-2, -0.5)}, Timestamp::fromSeconds(0));
    world = Test::TestUtil::setEnemyRobotPositions(
        world, {Point(4, 3), Point(0.5, -1.5), Point(2, 0)}, Timestamp::fromSeconds(0));
    Robot r1(17, Point(2, -1), Vector(0, 0), Angle::ofDegrees(25),
             AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot r2(18, Point(2, -2), Vector(1, -1), Angle::ofDegrees(160),
             AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot r3(19, Point(2, -3), Vector(-1, 1), Angle::ofDegrees(200),
             AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableEnemyTeam().updateRobots({r1, r2, r3});

    drawWorld(scene, world);
    if(first_draw_call) {
        main_widget->ai_visualization_graphics_view->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
        first_draw_call = false;
    }
}

void MainWidget::updateRobotStatusMessages() {
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> dist(0,10);

    int num_entries = dist(rng);

    std::vector<std::string> status_msgs;
    for(int i = 0; i < num_entries; i++) {
        int age = dist(rng);
        std::string msg  = "Robot " + std::to_string(age) + "  --  some error oh no. Number " + std::to_string(age);
        status_msgs.emplace_back(msg);
    }

    setRobotStatus(main_widget->robot_status_table_widget, status_msgs);
}

void MainWidget::setupSceneView(QGraphicsView* view, QGraphicsScene* scene, QOpenGLWidget* gl_widget) {
    view->setScene(scene);
    view->setDragMode(QGraphicsView::ScrollHandDrag);
    view->setBackgroundBrush(QBrush(Qt::darkGreen, Qt::SolidPattern));

    // Performance optimizations
    // https://stackoverflow.com/questions/43826317/how-to-optimize-qgraphicsviews-performance
    view->setInteractive(false);
    view->setOptimizationFlag(QGraphicsView::DontAdjustForAntialiasing);
    view->setCacheMode(QGraphicsView::CacheBackground);
    view->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);
    // Using an OpenGL widget with the view should help make use of the graphics card rather than doing CPU
    // drawing, which should take some load off the CPU and make things faster
    view->setViewport(gl_widget);

    // Invert the y-coordinates of the view.
    // We do this because Qt's default coordinate system for drawing is:
    // * positive x = "right"
    // * positive y = "down"
    // Our coordinate system defines positive y as being "up", so we invert the coordinate
    // system so all our draw calls can follow our coordinate convention. This also fixes the orientation
    // convention, so "positive" rotation is counterclockwise in the view (rotating from +x, to +y, to -x, to -y)
    QTransform view_transform(1, 0, 0, -1, 0, 0);
    view->setTransform(view_transform);

    view->update();
}

void MainWidget::setupStatusTable(QTableWidget* table) {
    QList<QString> horizontal_header_labels({"Age (Seconds)", "Message"});
    // The column count must be set before the labels are set
    table->setColumnCount(horizontal_header_labels.size());
    table->setHorizontalHeaderLabels(horizontal_header_labels);
    table->horizontalHeader()->setVisible(true);
    table->horizontalHeader()->setStretchLastSection(true);

    // Hide line numbers
    table->verticalHeader()->setVisible(false);
    table->setShowGrid(true);
    table->setVisible(true);

    table->update();
}

void MainWidget::setRobotStatus(QTableWidget* table, std::vector<std::string> robot_status_messages) {
    // Resize the number of rows to only have as many rows as we have messages. This will automatically
    // delete any extra rows / messages for us, and then we overwrite the existing rows with new messages
    table->setRowCount(robot_status_messages.size());
    for(int i = 0; i < robot_status_messages.size(); i++) {
        table->setItem(i, 0, new QTableWidgetItem("some age placeholder"));
        auto message_data = QString::fromStdString(robot_status_messages.at(i));
        table->setItem(i, 1, new QTableWidgetItem(message_data));
    }
}
