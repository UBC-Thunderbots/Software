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
    explicit ZoomableQGraphicsView(QWidget *parent = nullptr);

   protected slots:
    void wheelEvent(QWheelEvent *event);

   private:
    // The base of the exponential used to calculate how much to zoom
    // given a certain amount of mouse wheel movement
    static constexpr double ZOOM_SCALING_FACTOR_EXPONENTIAL_BASE = 1.0005;
};
