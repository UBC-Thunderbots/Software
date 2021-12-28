//#ifndef _DRAWER_H_
//#define _DRAWER_H_
//
//#include <string>
//#include "PathfindingDataTypes.h"
//
//namespace Pathfinding {
//
//class Grid;
//class TGAImage;
//namespace ENLSVG { class VisibilityGraph; }
//struct Colour;
//
//class Drawer {
//public:
//    Drawer(const Grid& grid, int scale=1);
//    ~Drawer();
//
//    void save(const std::string& filename);
//    void drawGrid(const Grid& grid);
//    void drawVisibilityGraph(const ENLSVG::VisibilityGraph& graph);
//    void drawPoint(int px, int py);
//    void drawPath(const Path& path);
//    void drawParentPointers(const ParentPtrs& ptrs);
//
//    void drawLine(int x1 ,int y1, int x2, int y2, const Colour& c);
//    void drawCircle(int cx, int cy, int r, const Colour& c);
//    void drawRect(int x1, int y1, int x2, int y2, const Colour& c);
//
//    // Don't change the order of variable declaration!
//    const int gridSizeX;
//    const int gridSizeY;
//    const int imgSizeX;
//    const int imgSizeY;
//    const int scale;
//private:
//    TGAImage* imgPtr;
//};
//}
//
//#endif