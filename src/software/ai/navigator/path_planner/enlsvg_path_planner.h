#include "extlibs/enlsvg/Pathfinding/ENLSVG.h" 
#include "path_planner.h"
#include "software/world/world.h"

class EnlsvgPathPlanner : public PathPlanner
{
    public:
        EnlsvgPathPlanner(const Rectangle &navigable_area, const std::vector<ObstaclePtr> &obstacles);   
        /**
         * Returns a path that is an optimized path between start and end.
         *
         * @param start start point
         * @param end end point
         * @param navigable_area Rectangle representing the navigable area
         * @param obstacles obstacles to avoid
         *
         * @return a vector of points that is the optimal path avoiding obstacles
         *         if no valid path then return empty vector
         */
        std::optional<Path> findPath(const Point &start, const Point &end,
                                     const Rectangle &navigable_area,
                                     const std::vector<ObstaclePtr> &obstacles) override;
        
    private:
        using GridVertex = Pathfinding::GridVertex;
        using GridPath = Pathfinding::Path;
    
        GridVertex convertPointToGridVertex(const Point &p) const;
        Point convertGridVertexToPoint(const GridVertex &gv) const;
        std::optional<Path> convertGridPathToPath(const GridPath &p) const;
        void createObstaclesInGrid(const std::vector<ObstaclePtr> &obstacles) const;
        bool isCoordNavigable(const GridVertex &gv) const;
    
        int num_grid_rows;
        int num_grid_cols;
        std::unique_ptr<const Pathfinding::ENLSVG::Algorithm> algo;
        std::unique_ptr<Pathfinding::ENLSVG::Memory> mem;
        std::unique_ptr<Pathfinding::Grid> grid;
        
        static constexpr double SIZE_OF_GRID_CELL_IN_METERS = 0.09; 
};