//#include <algorithm>
//#include "Drawer.h"
//#include "Grid.h"
//#include "Image.h"
//#include "ENLSVGGraph.h"
//
//namespace Pathfinding {
//
//Drawer::Drawer(const Grid& grid, int scale)
//: gridSizeX(grid.sizeX), gridSizeY(grid.sizeY), imgSizeX(grid.sizeX*scale),
//imgSizeY(grid.sizeY*scale), scale(scale), imgPtr(new TGAImage(imgSizeX, imgSizeY)) {
//}
//
//Drawer::~Drawer() {
//    delete imgPtr;
//}
//
//void Drawer::save(const std::string& filename) {
//    imgPtr->WriteImage(filename);
//}
//
//void Drawer::drawLine(int x1 ,int y1, int x2, int y2, const Colour& c) {
//    TGAImage& img = *imgPtr;
//    
//    int dy = y2 - y1;
//    int dx = x2 - x1;
//
//    int f = 0;
//
//    int signY = 1;
//    int signX = 1;
//    int offsetX = 0;
//    int offsetY = 0;
//    
//    if (dy < 0) {
//        dy *= -1;
//        signY = -1;
//        offsetY = -1;
//    }
//    if (dx < 0) {
//        dx *= -1;
//        signX = -1;
//        offsetX = -1;
//    }
//    
//    if (dx >= dy) {
//        while (x1 != x2) {
//            f += dy;
//            if (f >= dx) {
//                img.setPixelSafe(c, x1 + offsetX, y1 + offsetY);
//                y1 += signY;
//                f -= dx;
//            }
//            if (f != 0) img.setPixelSafe(c, x1 + offsetX, y1 + offsetY);
//            if (dy == 0) {
//                img.setPixelSafe(c, x1 + offsetX, y1);
//                img.setPixelSafe(c, x1 + offsetX, y1 - 1);
//            }
//
//            x1 += signX;
//        }
//    }
//    else {
//        while (y1 != y2) {
//            f += dx;
//            if (f >= dy) {
//                img.setPixelSafe(c, x1 + offsetX, y1 + offsetY);
//                x1 += signX;
//                f -= dy;
//            }
//            if (f != 0) img.setPixelSafe(c, x1 + offsetX, y1 + offsetY);
//            if (dx == 0) {
//                img.setPixelSafe(c, x1, y1 + offsetY);
//                img.setPixelSafe(c, x1 - 1, y1 + offsetY);
//            }
//
//            y1 += signY;
//        }
//    }
//}
//
//void Drawer::drawCircle(int cx, int cy, int r, const Colour& c) {
//    TGAImage& img = *imgPtr;
//    
//    for (int y=0;y<=r;++y) {
//        for (int x=0;x<=r;++x) {
//            if (x*x+y*y <= r*r) {
//                img.setPixelSafe(c,cx+x,cy+y);
//                img.setPixelSafe(c,cx-x,cy+y);
//                img.setPixelSafe(c,cx+x,cy-y);
//                img.setPixelSafe(c,cx-x,cy-y);
//            }
//        }
//    }
//}
//
//void Drawer::drawRect(int x1, int y1, int x2, int y2, const Colour& c) {
//    TGAImage& img = *imgPtr;
//
//    for (int y=y1;y<y2;++y) {
//        for (int x=x1;x<=x2;++x) {
//            img.setPixelSafe(c,x,y);
//        }
//    }
//}
//
//void Drawer::drawGrid(const Grid& grid) {
//    TGAImage& img = *imgPtr;
//
//    Colour c;
//    for (int y=0; y<gridSizeY; ++y) {
//        for (int x=0; x<gridSizeX; ++x) {
//            c.r = c.g = c.b = (grid.isBlockedRaw(x,y) ? 64 : 255);
//            int x1 = x*scale;
//            int x2 = x1+scale;
//            int y1 = y*scale;
//            int y2 = y1+scale;
//            for (int j=y1;j<y2;++j) {
//                for (int i=x1;i<x2;++i) {
//                    img.setPixel(c,i,j);
//                }
//            }
//        }
//    }
//}
//
//void Drawer::drawVisibilityGraph(const ENLSVG::VisibilityGraph& graph) {
//    TGAImage& img = *imgPtr;
//
//    Colour cols[] {
//        Colours::BLUE,
//        Colours::LIME,
//        Colours::GREEN,
//        Colours::YELLOW,
//        Colours::ORANGE,
//        Colours::RED,
//        Colours::PURPLE,
//        Colours::BROWN,
//        Colours::CYAN,
//        Colours::PINK,
//        Colours::AQUA,
//        Colours::TEAL,
//        Colours::GREY,
//        Colours::LIGHTGREY
//    };
//    int nCols = sizeof(cols) / sizeof(*cols);
//
//    const int LEVEL_W = graph.LEVEL_W;
//
//    //Colour c = Colours::LIME;
//    const std::vector<ENLSVG::EdgeData>& edges = graph.edges;
//    const std::vector<GridVertex>& vertices = graph.vertices;
//    
//    // Do a counting sort on the edge indexes by level.
//    // MAX LEVEL <= edges.size()+1 (safe bound)
//    std::vector<size_t> iterationOrder;
//    {
//        const size_t levelWslot = edges.size()+1;
//        std::vector<int> levelCounts;
//        levelCounts.resize(edges.size()+3, 0);
//        for (size_t i=0; i<edges.size(); ++i) {
//            int level = edges[i].level;
//            levelCounts[(level != LEVEL_W ? level : levelWslot)+1]++;
//        }
//        for (size_t i=1; i<levelCounts.size(); ++i) {
//            levelCounts[i] += levelCounts[i-1];
//        }
//        iterationOrder.resize(edges.size());
//        for (size_t i=0; i<edges.size(); ++i) {
//            int level = edges[i].level;
//            level = level != LEVEL_W ? level : levelWslot;
//            iterationOrder[levelCounts[level]++] = i;
//        }
//    }
//
//
//    for (size_t i=0; i<iterationOrder.size(); ++i) {
//        const ENLSVG::EdgeData& edge = edges[iterationOrder[i]];
//        const GridVertex& u = vertices[edge.sourceVertex];
//        const GridVertex& v = vertices[edge.destVertex];
//
//        const Colour& c = edge.level == -1 ? Colours::BLACK : cols[std::min(edge.level,nCols)];
//
//        drawLine(u.x*scale, u.y*scale, v.x*scale, v.y*scale, c);
//    }
//
//    for (size_t i=0; i<vertices.size(); ++i) {
//        if (graph.skipEdges[i].size() == 0) continue;
//
//        const Colour& c = Colours::MAGENTA;
//        const GridVertex& u = vertices[i];
//        const std::vector<ENLSVG::SkipEdge>& edgeList = graph.skipEdges[i];
//        for (size_t j=0; j<edgeList.size(); ++j) {
//            const GridVertex& v = vertices[edgeList[j].next];
//            drawLine(u.x*scale, u.y*scale, v.x*scale, v.y*scale, c);
//        }
//    }
//
//    for (size_t i=0; i<vertices.size(); ++i) {
//        if (graph.skipEdges[i].size() == 0) continue;
//        const Colour& c = Colours::BLACK;
//        drawCircle(vertices[i].x*scale, vertices[i].y*scale, 3, c);
//    }
//}
//
//void Drawer::drawPoint(int px, int py) {
//    const Colour& c1 = Colours::BROWN;
//    drawCircle(px*scale, py*scale, 6, c1);
//
//    const Colour& c2 = Colours::YELLOW;
//    drawCircle(px*scale, py*scale, 4, c2);
//}
//
//
//void Drawer::drawPath(const Path& path) {
//    if (path.size() == 0) return;
//
//    int prevX = path[0].x;
//    int prevY = path[0].y;
//    int currX, currY;
//
//    const Colour& c = Colours::BLUE;
//    for (size_t i=1; i<path.size(); ++i) {
//        currX = path[i].x;
//        currY = path[i].y;
//
//        drawLine(prevX*scale, prevY*scale, currX*scale, currY*scale, c);
//        
//        prevX = currX;
//        prevY = currY;
//    }
//}
//
//
//void Drawer::drawParentPointers(const ParentPtrs& ptrs) {
//
//    const Colour& c = Colours::ORANGE;
//    int x1,y1,x2,y2;
//
//    x1 = ptrs.goal.x;
//    y1 = ptrs.goal.y;
//    x2 = ptrs.goalParent.x;
//    y2 = ptrs.goalParent.y;
//    drawLine(x1*scale, y1*scale, x2*scale, y2*scale, c);
//
//    const std::vector<GridVertex>& current = ptrs.current;
//    const std::vector<GridVertex>& parent = ptrs.parent;
//    for (size_t i=0; i<current.size(); ++i) {
//        x1 = current[i].x;
//        y1 = current[i].y;
//        x2 = parent[i].x;
//        y2 = parent[i].y;
//        drawLine(x1*scale, y1*scale, x2*scale, y2*scale, c);
//    }
//}
//}