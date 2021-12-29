#include "extlibs/enlsvg/Pathfinding/ENLSVG.h" 
#include "path_planner.h"
#include "software/world/world.h"

class GlobalPathPlanner : public PathPlanner
{
    public:
        GlobalPathPlanner(const Rectangle &navigable_area, const World &world);
        
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
        Pathfinding::ENLSVG::Algorithm algo;
        Pathfinding::ENLSVG::Memory mem;
        int num_grid_rows;
        int num_grid_cols;
        Pathfinding::Grid grid;
        
        static constexpr int RESOLUTION_IN_CM = 9; 
};