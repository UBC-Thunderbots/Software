#include "software/gui/generic_widgets/draw_function_visualizer/zoomable_qgraphics_view.h"

#include <QtCore/QtMath>

ZoomableQGraphicsView::ZoomableQGraphicsView(QWidget *parent) : QGraphicsView(parent) {}

void ZoomableQGraphicsView::wheelEvent(QWheelEvent *event)
{
    // Do a mouse-wheel based zoom about the cursor position
    // See https://stackoverflow.com/a/44422044
    double wheel_angle_delta = event->angleDelta().y();
    double scaling_factor = qPow(ZOOM_SCALING_FACTOR_EXPONENTIAL_BASE, wheel_angle_delta);

    auto target_viewport_pos = event->pos();
    auto target_scene_pos    = mapToScene(event->pos());

    scale(scaling_factor, scaling_factor);
    centerOn(target_scene_pos);
    QPointF delta_viewport_pos =
        target_viewport_pos -
        QPointF(viewport()->width() / 2.0, viewport()->height() / 2.0);
    QPointF viewport_center = mapFromScene(target_scene_pos) - delta_viewport_pos;
    centerOn(mapToScene(viewport_center.toPoint()));
}
