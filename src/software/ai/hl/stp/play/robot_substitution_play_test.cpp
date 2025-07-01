#include <gtest/gtest.h>
#include <algorithm>
#include <exception>
#include "software/ai/hl/stp/play/halt_play/halt_play.h"
#include "software/ai/hl/stp/play/offense/offense_play.h"
#include "software/test_util/test_util.h"

std::shared_ptr<World> createRobotSubstitutionWorld(){
    Team friendly_team(Duration::fromSeconds(0));
    Robot robot_0(0, Point(-1.1, 1), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_1(1, Point(2, 0.81), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    Robot robot_2(2, Point(0, 5.0), Vector(), Angle::zero(), AngularVelocity::zero(),
                  Timestamp::fromSeconds(0));
    friendly_team.updateRobots({robot_0, robot_1, robot_2});
    
    // Setting injured robot to be the first robot
    std::vector<Robot> injured_robots;
    injured_robots.push_back(robot_1);
    friendly_team.setInjuredRobots(injured_robots);

    std::shared_ptr<World> world = TestUtil::createBlankTestingWorld();
    world->updateFriendlyTeamState(friendly_team);
    return world;
}

/**
 * Given that there is one robot on the injured on the field, check if 
 * we can make the substitution on the positive side
 */
TEST(TacticAssignmentTest, test_positive_side)
{
    // Setting the team state
    std::shared_ptr<World> world = createRobotSubstitutionWorld();

    // setting the play
    TbotsProto::AiConfig ai_config;
    OffensePlay play(ai_config);
    auto robot_navigation_obstacle_config = ai_config.robot_navigation_obstacle_config();
    InterPlayCommunication comm;
    auto p_set = play.get(world, comm, [this](InterPlayCommunication comm) {});

    // checking if we have a move primitive at
    TbotsProto::Primitive injured_primitive = p_set->robot_primitives().at(1);
    Point expected_pos(0, world->field().totalYLength() / 2);
    Point injured_robot_pos =
        createPoint(injured_primitive.move().xy_traj_params().destination());
    EXPECT_EQ(expected_pos, injured_robot_pos);
}

/**
 * Given that there is one robot on the injured on the field, check if 
 * we can make the substitution on the negative side
 */
TEST(TacticAssignmentTest, test_negative_side)
{
    // Setting the team state
    std::shared_ptr<World> world = createRobotSubstitutionWorld();

    // setting the play
    TbotsProto::AiConfig ai_config;
    TbotsProto::RobotSubstitutionConfig substitution_config;
    ai_config.mutable_robot_substitution_config()->set_substitute_at_positive_y(false);

    OffensePlay play(ai_config);
    auto robot_navigation_obstacle_config = ai_config.robot_navigation_obstacle_config();
    InterPlayCommunication comm;
    auto p_set = play.get(world, comm, [this](InterPlayCommunication comm) {});

    // checking if we have a move primitive at
    TbotsProto::Primitive injured_primitive = p_set->robot_primitives().at(1);
    Point expected_pos(0, -world->field().totalYLength() / 2);
    Point injured_robot_pos =
        createPoint(injured_primitive.move().xy_traj_params().destination());
    EXPECT_EQ(expected_pos, injured_robot_pos);
}
