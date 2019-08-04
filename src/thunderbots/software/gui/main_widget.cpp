#include "software/gui/main_widget.h"
#include "ui_main_widget.h" // TODO: find the real path
#include <iostream>
#include <random>
#include "software/gui/draw_world.h"


MainWidget::MainWidget(QWidget *parent) :
        QWidget(parent),
        main_widget(new Ui::MainWidget()){
    // Handles all the setup of the generated UI components and adds the components
    // to this widget
    main_widget->setupUi(this);

    scene = new QGraphicsScene(this); // TODO: make the parent the graphics view?
    glWidget = new QOpenGLWidget(this);
    qglWidget = new QGLWidget(QGLFormat(QGL::SampleBuffers | QGL::DirectRendering), this);
    main_widget->graphicsView->setScene(scene);
    main_widget->graphicsView->setViewport(qglWidget);
    // TODO: What does this do?
    main_widget->graphicsView->setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
//    main_widget->graphicsView->ev
//    main_widget->graphicsView->setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);

    // Optimizations
    // https://stackoverflow.com/questions/43826317/how-to-optimize-qgraphicsviews-performance
//    main_widget->graphicsView->setInteractive(false);
//    main_widget->graphicsView->setOptimizationFlag(QGraphicsView::DontAdjustForAntialiasing);
    main_widget->graphicsView->setCacheMode(QGraphicsView::CacheBackground);
    main_widget->graphicsView->setDragMode(QGraphicsView::ScrollHandDrag);

    // Set the background color to green to look like a field
    main_widget->graphicsView->setBackgroundBrush(QBrush(Qt::darkGreen, Qt::SolidPattern));

    main_widget->graphicsView->fitInView(scene->sceneRect(),Qt::KeepAspectRatio);
    // Apply updates?
    main_widget->graphicsView->update();

}

// TODO: Stop using pointers wherever possible, so I don't have to manually destruct stuff
MainWidget::~MainWidget() {
    delete main_widget;
}

void MainWidget::drawAI() {
    scene->clear();


    Field field = Field(9.0, 6.0, 1.0, 2.0, 1.0, 0.3, 0.5, Timestamp::fromSeconds(0));
    drawField(scene, field);

//    QBrush brush(QColor::fromRgb(255,127,80, 255));
//    QPen pen(Qt::black);
//    pen.setWidth(2);
//    scene->addLine(-100, 0, 100, 0);
//    scene->addLine(0, -100, 0, 100);
//
//    std::random_device rd; // obtain a random number from hardware
//    std::mt19937 eng(rd()); // seed the generator
//    std::uniform_int_distribution<> distr(-300, 300); // define the range
//
//    for(int i = 0; i < 1000; i++) {
//        int p1 = distr(eng);
//        int p2 = distr(eng);
//        scene->addEllipse(p1, p2, 50, 40, pen, brush);
//    }

}
