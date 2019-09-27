#pragma once

#include <QtGui/QKeyEvent>
#include <QtGui/QWheelEvent>
#include <QtWidgets/QGraphicsView>

/**
 * A custom QGraphicsView that allows zooming with the mousewheel
 */
class ZoomableQGraphicsView : public QGraphicsView
{
    Q_OBJECT
   public:
    explicit ZoomableQGraphicsView(QWidget *parent = 0);

   protected slots:
    void wheelEvent(QWheelEvent *event);
};
