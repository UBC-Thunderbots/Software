#include "gui/SceneView.h"

MySceneView::MySceneView(QWidget *parent) : QGraphicsView(parent)
{
    setDragMode(QGraphicsView::ScrollHandDrag);
    setCacheMode(QGraphicsView::CacheBackground);
    setBackgroundBrush(QBrush(Qt::darkGreen, Qt::SolidPattern));

//    QGraphicsScene *scene = new QGraphicsScene();

//    QPen pen(Qt::white);
//    pen.setWidth(1);
//    pen.setCosmetic(true);
//    scene->addRect(-9, 6, 9, -6);

//    setScene(scene);

//    fitInView(scene->sceneRect(),Qt::KeepAspectRatio);
//    update();
}

void MySceneView::wheelEvent(QWheelEvent *event)
    {
        if(event->delta() > 0)
            scale(1.1, 1.1);
        else
            scale(0.95, 0.95);
    }

    void MySceneView::keyPressEvent(QKeyEvent *event)
    {
        if(event->key() == Qt::Key_Left)
            rotate(1);
        else if(event->key() == Qt::Key_Right)
            rotate(-1);
    }
