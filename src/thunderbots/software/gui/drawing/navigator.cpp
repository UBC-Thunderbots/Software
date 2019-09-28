#include "software/gui/drawing/navigator.h"

#include <QtWidgets/QGraphicsScene>

#include "software/geom/segment.h"
#include "software/gui/geom/geometry_conversion.h"

AIDrawFunction drawNavigator(std::shared_ptr<Navigator> navigator)
{
    auto planned_paths = navigator->getPlannedPaths();
    auto draw_function = [planned_paths](QGraphicsScene* scene) {
        QPen pen(Qt::darkBlue);
        // The cap style must be NOT be set to SquareCap. It can be set to anything else.
        // Drawing a line of length 0 with the SquareCap style causes a large line to be
        // drawn
        pen.setCapStyle(Qt::PenCapStyle::RoundCap);
        pen.setWidth(2);
        pen.setCosmetic(true);

        QBrush brush(Qt::darkBlue);
        brush.setStyle(Qt::BrushStyle::SolidPattern);

        for (const auto& path : planned_paths)
        {
            for (auto i = 0; i < path.size() - 1; i++)
            {
                Segment path_segment(path[i], path[i + 1]);
                QLineF line = createQLineF(path_segment);
                scene->addLine(line, pen);
            }
        }
    };

    return AIDrawFunction(draw_function);
}
