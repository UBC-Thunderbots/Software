#include "software/gui/drawing/navigator.h"
#include <QtWidgets/QGraphicsScene>

DrawFunction drawNavigator(std::shared_ptr<Navigator> navigator) {
    return [navigator](QGraphicsScene* scene) {
        QPen pen(Qt::darkBlue);
        pen.setWidth(2);
        pen.setCosmetic(true);

        QBrush brush(Qt::darkBlue);
        brush.setStyle(Qt::BrushStyle::SolidPattern);

        scene->addEllipse(0, 0, 500, 500, pen, brush);
    };
}
