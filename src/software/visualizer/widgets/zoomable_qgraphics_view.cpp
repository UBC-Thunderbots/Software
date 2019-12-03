#include "software/gui/widgets/zoomable_qgraphics_view.h"

ZoomableQGraphicsView::ZoomableQGraphicsView(QWidget *parent) : QGraphicsView(parent) {}

void ZoomableQGraphicsView::wheelEvent(QWheelEvent *event)
{
    if (event->delta() > 0)
        scale(zoom_in_scaling_factor, zoom_in_scaling_factor);
    else
        scale(zoom_out_scaling_factor, zoom_out_scaling_factor);
}
