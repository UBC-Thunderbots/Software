#include "software/gui/main_widget.h"

#include <ai_gui_autogen/include/ai_gui_autogen/include/ui_main_widget.h>

#include <iostream>

#include "gui/drawing/world.h"
#include "test/test_util/test_util.h"
#include <random>

MainWidget::MainWidget(QWidget *parent)
    : QWidget(parent), main_widget(new Ui::MainWidget())
{
    // Handles all the setup of the generated UI components and adds the components
    // to this widget
    main_widget->setupUi(this);

    scene = new QGraphicsScene(main_widget->graphicsView);
    glWidget = new QOpenGLWidget(this);
    setupSceneView(main_widget->graphicsView, scene, glWidget);

    setupStatusTable(main_widget->myTableWidget);
    setRobotStatus(main_widget->myTableWidget, {"foo", "bar", "hallsensor"});

        // TODO: DO this only for the first draw call
//    view->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
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
    main_widget->graphicsView->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
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

    setRobotStatus(main_widget->myTableWidget, status_msgs);
}

void MainWidget::setupSceneView(QGraphicsView* view, QGraphicsScene* scene, QOpenGLWidget* gl_widget) {
    view->setScene(scene);
    view->setDragMode(QGraphicsView::ScrollHandDrag);

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
