#include "software/test_util/test_util.h"

TEST(PassingSimTest, test_pass)
{
    World world = ::TestUtil::createBlankTestingWorld();
    Robot robot1 = ::TestUtil::createRobotAtPos(Point(-1, 0));
    Robot robot2 = ::TestUtil::createRobotAtPos(Point(1, 0));


}