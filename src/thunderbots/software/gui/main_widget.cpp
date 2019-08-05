#include "software/gui/main_widget.h"
#include <ai_gui_autogen/include/ui_main_widget.h>
#include <iostream>
#include "gui/drawing/world.h"
#include "test/test_util/test_util.h"


MainWidget::MainWidget(QWidget *parent) :
        QWidget(parent),
        main_widget(new Ui::MainWidget()){
    // Handles all the setup of the generated UI components and adds the components
    // to this widget
    main_widget->setupUi(this);

    scene = new QGraphicsScene(main_widget->graphicsView);
//    glWidget = new QOpenGLWidget(this);
//    qglWidget = new QGLWidget(QGLFormat(QGL::SampleBuffers | QGL::DirectRendering), this);
    main_widget->graphicsView->setScene(scene);
//    main_widget->graphicsView->setViewport(qglWidget);
    // TODO: What does this do?
//    main_widget->graphicsView->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);

//    main_widget->splitter->addWidget(view);
//    main_widget->graphicsView->ev
//    main_widget->graphicsView->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);

    // Optimizations
    // https://stackoverflow.com/questions/43826317/how-to-optimize-qgraphicsviews-performance
//    main_widget->graphicsView->setInteractive(false);
//    main_widget->graphicsView->setOptimizationFlag(QGraphicsView::DontAdjustForAntialiasing);
//    main_widget->graphicsView->setCacheMode(QGraphicsView::CacheBackground);
//    main_widget->graphicsView->setDragMode(QGraphicsView::ScrollHandDrag);

    // Invert the y-coordinates of the view.
    // We do this because Qt's default coordinate system for drawing is:
    // * positive x = "right"
    // * positive y = "down"
    // Our coordinate system defines positive y as being "up", so we invert the coordinate system so all our
    // draw calls can follow our coordinate convention
    main_widget->graphicsView->scale(1, -1);
    main_widget->graphicsView->fitInView(scene->sceneRect(),Qt::KeepAspectRatio);
    main_widget->graphicsView->update();

}

// TODO: Stop using pointers wherever possible, so I don't have to manually destruct stuff
MainWidget::~MainWidget() {
    delete main_widget;
}

void MainWidget::drawAI() {
    scene->clear();

    World world = Test::TestUtil::createBlankTestingWorld();
    world = Test::TestUtil::setBallPosition(world, Point(-1, 1), Timestamp::fromSeconds(0));
    world = Test::TestUtil::setBallVelocity(world, Vector(1, -1.5), Timestamp::fromSeconds(0));
    world = Test::TestUtil::setFriendlyRobotPositions(world, {Point(-4, 1), Point(0, -3), Point(-2, -0.5)}, Timestamp::fromSeconds(0));
//    world = Test::TestUtil::setEnemyRobotPositions(world, {Point(4, 3), Point(0.5, -1.5), Point(2, 0)}, Timestamp::fromSeconds(0));
    Robot r1(17, Point(2, -1), Vector(1, 1), Angle::ofDegrees(25), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot r2(18, Point(2, -2), Vector(1, -1), Angle::ofDegrees(160), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    Robot r3(19, Point(2, -3), Vector(-1, 1), Angle::ofDegrees(200), AngularVelocity::zero(), Timestamp::fromSeconds(0));
    world.mutableEnemyTeam().updateRobots({r1, r2, r3});

    drawWorld(scene, world);
//    main_widget->graphicsView->fitInView(scene->sceneRect(),Qt::KeepAspectRatio);
}
