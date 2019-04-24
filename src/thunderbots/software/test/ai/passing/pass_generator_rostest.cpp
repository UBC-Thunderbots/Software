#include <gtest/gtest.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>

#include <string>

#include "test/test_util/rostest_util.h"
#include "test/test_util/test_util.h"
#include "util/logger/init.h"
#include "util/canvas_messenger/canvas_messenger.h"
#include "ai/passing/pass_generator.h"
#include "ai/world/world.h"

class PassGeneratorRosTest : public testing::Test
{
   protected:
    virtual void SetUp()
    {
        // Initialize the draw visualizer messenger
        Util::CanvasMessenger::getInstance()->initializePublisher(nh_);
    }

    ros::NodeHandle nh_;
};

TEST_F(PassGeneratorRosTest, deleteme){
    World world = ::Test::TestUtil::createBlankTestingWorld();

    Team new_friendly_team(Duration::fromSeconds(0.1));
    new_friendly_team.updateRobots({
                Robot(0, Point(1, 0), Point(0,0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0.3)),
                Robot(1, Point(2, 2), Point(0,0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0.3))
            });
    world.updateFriendlyTeamState(new_friendly_team);

    Team new_enemy_team(Duration::fromSeconds(0.1));
    new_enemy_team.updateRobots({
        Robot(0, Point(2, 0.0), Point(0,0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0.3)),
        Robot(1, Point(2, 0.3), Point(0,0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0.3)),
        Robot(2, Point(2, -0.3), Point(0,0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0.3)),
        Robot(3, Point(0, -2), Point(0,0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0.3)),
        Robot(4, Point(0, 2), Point(0,0), Angle::zero(), AngularVelocity::zero(), Timestamp::fromSeconds(0.3)),
                                });
    world.updateEnemyTeamState(new_enemy_team);


    AI::Passing::PassGenerator pass_generator(0.0);

    do
    {
        // Display the world
        Util::CanvasMessenger::getInstance()->drawField(world.field());
        for(const auto& r : world.friendlyTeam().getAllRobots()) {
            Util::CanvasMessenger::getInstance()->drawPoint(r.position(), 0.2, 0, 100, 0, 255);
        }
        for(const auto& r : world.enemyTeam().getAllRobots()) {
            Util::CanvasMessenger::getInstance()->drawPoint(r.position(), 0.2, 100, 0, 0, 255);
        }
        Util::CanvasMessenger::getInstance()->drawPoint(world.ball().position(), 0.15, 255, 140, 0, 255);

        pass_generator.setWorld(world);
        std::this_thread::yield();
    } while (true);
}

int main(int argc, char **argv)
{
    // !! Don't forget to initialize ROS, since this is a test within the ros framework !!
    ros::init(argc, argv, "logger_test");
    ::testing::InitGoogleTest(&argc, argv);
    // Because of https://github.com/ros/ros_comm/issues/688 we create an extra NodeHandle
    // here that will stay in scope until all the tests have been completed. Otherwise
    // once the NodeHandle in the first test that is run goes out of scope, the rosconsole
    // functions (ROS_INFO, etc.) will no longer work, and therefore the logger will no
    // longer work and the tests will fail
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}
