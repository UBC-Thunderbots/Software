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

private:
    const double zoom_in_scaling_factor = 1.04;
    const double zoom_out_scaling_factor = 0.96;
};
