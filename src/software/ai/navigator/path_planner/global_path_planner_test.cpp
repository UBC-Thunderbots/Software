#include "software/test_util/test_util.h"
#include "global_path_planner.h"
#include "shared/parameter/cpp_dynamic_parameters.h"

class TestGlobalPathPlanner : public testing::Test
{
    public:
        TestGlobalPathPlanner()
            : world(TestUtil::createBlankTestingWorldDivA()),
              gpp(std::make_shared<RobotNavigationObstacleConfig>(), world, world.field().fieldBoundary())
        {
        }
        
    protected:
        World world;
        GlobalPathPlanner gpp;
};

TEST_F(TestGlobalPathPlanner, test_no_motion_constraints)
{
    Point start{1, 2}, dest {3, -2};
    
    std::shared_ptr<EnlsvgPathPlanner> planner = gpp.getPathGenerator(world, {});
}
