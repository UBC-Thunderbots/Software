#include "gui/SceneView.h"

SceneView::SceneView(QWidget *parent) : QGraphicsView(parent)
{
    setDragMode(QGraphicsView::ScrollHandDrag);
    setCacheMode(QGraphicsView::CacheBackground);
    setBackgroundBrush(QBrush(Qt::darkGreen, Qt::SolidPattern));

    QGraphicsScene *scene = new QGraphicsScene();


    setScene(scene);

    fitInView(scene->sceneRect(),Qt::KeepAspectRatio);
    update()
}

void SceneView::wheelEvent(QWheelEvent *event)
    {
        if(event->delta() > 0)
            scale(1.25, 1.25);
        else
            scale(0.8, 0.8);
    }

    void SceneView::keyPressEvent(QKeyEvent *event)
    {
        if(event->key() == Qt::Key_Left)
            rotate(1);
        else if(event->key() == Qt::Key_Right)
            rotate(-1);
    }
