#include "extlibs/enlsvg/Pathfinding/ENLSVG.h" 
#include "path_planner.h"
#include "software/world/world.h"

#include <queue>

class EnlsvgPathPlanner : public PathPlanner
{
    public:
        EnlsvgPathPlanner(const Rectangle &navigable_area, const std::vector<ObstaclePtr> &obstacles,
                        double boundary_margin_offset);   
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
                                     
        inline double getResolution() const { return SIZE_OF_GRID_CELL_IN_METERS; }
        
    private:
        using EnlsvgPoint       = Pathfinding::GridVertex;
        using EnlsvgPath        = Pathfinding::Path;
        using EnlsvgGrid        = Pathfinding::Grid;
        using EnlsvgAlgorithm   = Pathfinding::ENLSVG::Algorithm;
        using EnlsvgMemory      = Pathfinding::ENLSVG::Memory;
    
        EnlsvgPoint convertPointToEnlsvgPoint(const Point &p) const;
        Point convertEnlsvgPointToPoint(const EnlsvgPoint &gv) const;
        std::optional<Path> convertEnlsvgPathToPath(const EnlsvgPath &p) const;
        void createObstaclesInGrid(const std::vector<ObstaclePtr> &obstacles, double grid_boundary_margin_offset) const;
        bool isCoordNavigable(const EnlsvgPoint &gv) const;
        std::optional<EnlsvgPoint> findClosestUnblockedEnlsvgPoint(const EnlsvgPoint &ep) const;
        bool isBlocked(const EnlsvgPoint &ep) const;
        void blockNearbyCoordDueToRobotRadius(const Point &point) const;
    
        int num_grid_rows;
        int num_grid_cols;
//        int min_navigable_y_enlsvg_point;
//        int min_navigable_x_enlsvg_point;
        Point origin;
        int max_navigable_y_enlsvg_point;
        int max_navigable_x_enlsvg_point;
        std::unique_ptr<EnlsvgGrid> grid;
        std::unique_ptr<const EnlsvgAlgorithm> algo;
        std::unique_ptr<EnlsvgMemory> mem;
        
        // analogous to resolution of grid
        static constexpr double SIZE_OF_GRID_CELL_IN_METERS = 0.09; 
};