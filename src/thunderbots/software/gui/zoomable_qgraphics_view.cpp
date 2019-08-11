#include "gui/zoomable_qgraphics_view.h"

// No special behaviour beyond the regular QGraphicsView. All configuration should
// be done by the caller that owns instances of this object
ZoomableQGraphicsView::ZoomableQGraphicsView(QWidget *parent) : QGraphicsView(parent)
{}

void ZoomableQGraphicsView::wheelEvent(QWheelEvent *event)
{
    if (event->delta() > 0)
        scale(1.075, 1.075);
    else
        scale(0.95, 0.95);
}
