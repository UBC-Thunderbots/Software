#include "software/gui/drawing/navigator.h"
#include "software/gui/geometry_conversion.h"

void drawRobotPaths(QGraphicsScene *scene, std::vector<std::vector<Point>> paths) {
    QPen pen(Qt::darkRed);
    pen.setWidth(2);
    // The cap style must be NOT be set to SquareCap. It can be set to anything else.
    // Drawing a line of length 0 with the SquareCap style causes a large line to be drawn
    pen.setCapStyle(Qt::PenCapStyle::RoundCap);
    pen.setCosmetic(true);

    for(const auto path : paths) {
        std::cout << "path length " << path.size() << std::endl;
        for(int i = 0; i < path.size()-1; i++) {
            Segment p(path[i], path[i+1]);
            scene->addLine(createQLineF(p), pen);
        }
    }
}
