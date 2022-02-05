#include "software/test_util/test_util.h"
#include "global_path_planner.h"

class TestGlobalPathPlanner : public testing::Test
{
    public:
        TestGlobalPathPlanner()
            : gpp(TestUtil::createBlankTestingWorldDivA)
        {
        }
        
    private:
        GlobalPathPlanner gpp;
}