#include "software/gui/drawing/navigator.h"
#include <QtWidgets/QGraphicsScene>
#include "software/geom/segment.h"
#include "software/gui/geom/geometry_conversion.h"

AIDrawFunction drawNavigator(std::shared_ptr<Navigator> navigator) {
    static unsigned int count = 0;
    count++;
    // Thought we were catching thenavigator between superclass and
    // subclass initialization. Apparently not....? Or at least this
    // isn't the entire problem
    if(count < 300) {
        return AIDrawFunction();
    }
    if(!navigator) {
        return AIDrawFunction();
    }
    auto planned_paths = navigator->getPlannedPaths();
    if(planned_paths.size() > 6) {
        std::cout << "BAD" << std::endl;
        return AIDrawFunction();
    }
    // TODO: You are here. Bad memory stuff when capturing paths
    // Try make this as close to the world drawing functions as possible?
    return AIDrawFunction();
    // capturing the planned_paths causes wild memory failures
    // from bad_alloc to corrupted double-linked list, to segfault
    // The segfault is from copying the planned paths, seems like it's
    // actually uninitialized
    auto draw_function = [planned_paths](QGraphicsScene* scene) {

        scene->addEllipse(0, 0, 10, 10, QPen(Qt::red), QBrush(Qt::transparent));


        QPen pen(Qt::darkBlue);
        // The cap style must be NOT be set to SquareCap. It can be set to anything else.
        // Drawing a line of length 0 with the SquareCap style causes a large line to be drawn
        pen.setCapStyle(Qt::PenCapStyle::RoundCap);
        pen.setWidth(2);
        pen.setCosmetic(true);

        QBrush brush(Qt::darkBlue);
        brush.setStyle(Qt::BrushStyle::SolidPattern);

//        for(const auto& path : planned_paths) {
//            for(auto i = 0; i < planned_paths.size()-1; i++) {
//                Segment path_segment(path[i], path[i+1]);
//                QLineF line = createQLineF(path_segment);
//                scene->addLine(line, pen);
//            }
//        }
    };

    return AIDrawFunction(draw_function);
}


