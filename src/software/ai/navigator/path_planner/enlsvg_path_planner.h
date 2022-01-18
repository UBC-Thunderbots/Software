#include "extlibs/enlsvg/Pathfinding/ENLSVG.h" 
#include "path_planner.h"
#include "software/world/world.h"

#include <queue>

/**
*   The Enlsvg Path Planner is a 
*/
class EnlsvgPathPlanner : public PathPlanner
{
    public:
        EnlsvgPathPlanner(const Rectangle &navigable_area, const std::vector<ObstaclePtr> &obstacles,
                        double boundary_margin_offset, double resolution = 0.09);   
        /**
         * Returns a path that is an optimized path between start and end.
         *
         * @param start start point
         * @param end end point
         * @param navigable_area Rectangle representing the navigable area
         * @param obstacles ignored
         *
         * @return a vector of points that is the optimal path avoiding obstacles
         *         if no valid path then return empty vector
         */
        std::optional<Path> findPath(const Point &start, const Point &end,
                                     const Rectangle &navigable_area,
                                     const std::vector<ObstaclePtr> &obstacles) override;
                                     
        inline double getResolution() const { return resolution; }
        
    private:
        using EnlsvgPoint       = Pathfinding::GridVertex;
        using EnlsvgPath        = Pathfinding::Path;
        using EnlsvgGrid        = Pathfinding::Grid;
        using EnlsvgAlgorithm   = Pathfinding::ENLSVG::Algorithm;
        using EnlsvgMemory      = Pathfinding::ENLSVG::Memory;
    
        // Conversion functions from Point <-> internal grid points
        EnlsvgPoint convertPointToEnlsvgPoint(const Point &p) const;
        Point convertEnlsvgPointToPoint(const EnlsvgPoint &gv) const;
        
        // 
        std::optional<Path> convertEnlsvgPathToPath(const EnlsvgPath &p) const;
        void createObstaclesInGrid(const std::vector<ObstaclePtr> &obstacles, double grid_boundary_margin_offset) const;
        bool isCoordNavigable(const EnlsvgPoint &gv) const;
        std::optional<EnlsvgPoint> findClosestUnblockedEnlsvgPoint(const EnlsvgPoint &ep) const;
        bool isBlocked(const EnlsvgPoint &ep) const;
        void blockNearbyCoordDueToRobotRadius(const Point &point) const;
            
        // analagous to size of grid
        double resolution;
    
        // rows: defined on x axis
        int num_grid_rows;
        // cols: defined on y axis
        int num_grid_cols;
        
        Point origin;
        
        int max_navigable_y_enlsvg_point;
        int max_navigable_x_enlsvg_point;

        // Required internal data structures
        std::unique_ptr<EnlsvgGrid> grid;
        std::unique_ptr<const EnlsvgAlgorithm> algo;
        std::unique_ptr<EnlsvgMemory> mem;
};