#pragma once

#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QKeyEvent>
#include <QWheelEvent>

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
